/*******************************************************************************
@File
@Title          Server bridge for rgxcmp
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for rgxcmp
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*******************************************************************************/

#include <linux/uaccess.h>

#include "img_defs.h"

#include "rgxcompute.h"

#include "common_rgxcmp_bridge.h"

#include "allocmem.h"
#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#if defined(SUPPORT_RGX)
#include "rgx_bridge.h"
#endif
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

#include "rgx_bvnc_defs_km.h"

/* ***************************************************************************
 * Server-side bridge entry points
 */

static_assert(RGXFWIF_RF_CMD_SIZE <= IMG_UINT32_MAX,
	      "RGXFWIF_RF_CMD_SIZE must not be larger than IMG_UINT32_MAX");
static_assert(RGXFWIF_STATIC_COMPUTECONTEXT_SIZE <= IMG_UINT32_MAX,
	      "RGXFWIF_STATIC_COMPUTECONTEXT_SIZE must not be larger than IMG_UINT32_MAX");

static IMG_INT
PVRSRVBridgeRGXCreateComputeContext(IMG_UINT32 ui32DispatchTableEntry,
				    PVRSRV_BRIDGE_IN_RGXCREATECOMPUTECONTEXT *
				    psRGXCreateComputeContextIN,
				    PVRSRV_BRIDGE_OUT_RGXCREATECOMPUTECONTEXT *
				    psRGXCreateComputeContextOUT,
				    CONNECTION_DATA * psConnection)
{
	IMG_BYTE *psFrameworkCmdInt = NULL;
	IMG_HANDLE hPrivData = psRGXCreateComputeContextIN->hPrivData;
	IMG_HANDLE hPrivDataInt = NULL;
	IMG_BYTE *psStaticComputecontextStateInt = NULL;
	RGX_SERVER_COMPUTE_CONTEXT *psComputeContextInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 0;
	IMG_UINT64 ui64BufferSize =
	    ((IMG_UINT64) psRGXCreateComputeContextIN->ui32FrameworkCmdize *
	     sizeof(IMG_BYTE)) +
	    ((IMG_UINT64) psRGXCreateComputeContextIN->
	     ui32StaticComputecontextStateSize * sizeof(IMG_BYTE)) + 0;

	if (unlikely
	    (psRGXCreateComputeContextIN->ui32FrameworkCmdize >
	     RGXFWIF_RF_CMD_SIZE))
	{
		psRGXCreateComputeContextOUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXCreateComputeContext_exit;
	}

	if (unlikely
	    (psRGXCreateComputeContextIN->ui32StaticComputecontextStateSize >
	     RGXFWIF_STATIC_COMPUTECONTEXT_SIZE))
	{
		psRGXCreateComputeContextOUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXCreateComputeContext_exit;
	}

	{
		PVRSRV_DEVICE_NODE *psDeviceNode = OSGetDevData(psConnection);

		/* Check that device supports the required feature */
		if ((psDeviceNode->pfnCheckDeviceFeature) &&
		    !psDeviceNode->pfnCheckDeviceFeature(psDeviceNode,
							 RGX_FEATURE_COMPUTE_BIT_MASK))
		{
			psRGXCreateComputeContextOUT->eError =
			    PVRSRV_ERROR_NOT_SUPPORTED;

			goto RGXCreateComputeContext_exit;
		}
	}

	if (ui64BufferSize > IMG_UINT32_MAX)
	{
		psRGXCreateComputeContextOUT->eError =
		    PVRSRV_ERROR_BRIDGE_BUFFER_TOO_SMALL;
		goto RGXCreateComputeContext_exit;
	}

	ui32BufferSize = (IMG_UINT32) ui64BufferSize;

	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset =
		    PVR_ALIGN(sizeof(*psRGXCreateComputeContextIN),
			      sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize =
		    ui32InBufferOffset >=
		    PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 : PVRSRV_MAX_BRIDGE_IN_SIZE -
		    ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer =
			    (IMG_BYTE *) psRGXCreateComputeContextIN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if (!pArrayArgsBuffer)
			{
				psRGXCreateComputeContextOUT->eError =
				    PVRSRV_ERROR_OUT_OF_MEMORY;
				goto RGXCreateComputeContext_exit;
			}
		}
	}

	if (psRGXCreateComputeContextIN->ui32FrameworkCmdize != 0)
	{
		psFrameworkCmdInt =
		    (IMG_BYTE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				  ui32NextOffset);
		ui32NextOffset +=
		    psRGXCreateComputeContextIN->ui32FrameworkCmdize *
		    sizeof(IMG_BYTE);
	}

	/* Copy the data over */
	if (psRGXCreateComputeContextIN->ui32FrameworkCmdize *
	    sizeof(IMG_BYTE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, psFrameworkCmdInt,
		     (const void __user *)psRGXCreateComputeContextIN->
		     psFrameworkCmd,
		     psRGXCreateComputeContextIN->ui32FrameworkCmdize *
		     sizeof(IMG_BYTE)) != PVRSRV_OK)
		{
			psRGXCreateComputeContextOUT->eError =
			    PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXCreateComputeContext_exit;
		}
	}
	if (psRGXCreateComputeContextIN->ui32StaticComputecontextStateSize != 0)
	{
		psStaticComputecontextStateInt =
		    (IMG_BYTE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				  ui32NextOffset);
		ui32NextOffset +=
		    psRGXCreateComputeContextIN->
		    ui32StaticComputecontextStateSize * sizeof(IMG_BYTE);
	}

	/* Copy the data over */
	if (psRGXCreateComputeContextIN->ui32StaticComputecontextStateSize *
	    sizeof(IMG_BYTE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, psStaticComputecontextStateInt,
		     (const void __user *)psRGXCreateComputeContextIN->
		     psStaticComputecontextState,
		     psRGXCreateComputeContextIN->
		     ui32StaticComputecontextStateSize * sizeof(IMG_BYTE)) !=
		    PVRSRV_OK)
		{
			psRGXCreateComputeContextOUT->eError =
			    PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXCreateComputeContext_exit;
		}
	}

	/* Lock over handle lookup. */
	LockHandle(psConnection->psHandleBase);

	/* Look up the address from the handle */
	psRGXCreateComputeContextOUT->eError =
	    PVRSRVLookupHandleUnlocked(psConnection->psHandleBase,
				       (void **)&hPrivDataInt,
				       hPrivData,
				       PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA,
				       IMG_TRUE);
	if (unlikely(psRGXCreateComputeContextOUT->eError != PVRSRV_OK))
	{
		UnlockHandle(psConnection->psHandleBase);
		goto RGXCreateComputeContext_exit;
	}
	/* Release now we have looked up handles. */
	UnlockHandle(psConnection->psHandleBase);

	psRGXCreateComputeContextOUT->eError =
	    PVRSRVRGXCreateComputeContextKM(psConnection,
					    OSGetDevData(psConnection),
					    psRGXCreateComputeContextIN->
					    ui32Priority,
					    psRGXCreateComputeContextIN->
					    ui32FrameworkCmdize,
					    psFrameworkCmdInt, hPrivDataInt,
					    psRGXCreateComputeContextIN->
					    sResumeSignalAddr,
					    psRGXCreateComputeContextIN->
					    ui32StaticComputecontextStateSize,
					    psStaticComputecontextStateInt,
					    psRGXCreateComputeContextIN->
					    ui32PackedCCBSizeU88,
					    &psComputeContextInt);
	/* Exit early if bridged call fails */
	if (unlikely(psRGXCreateComputeContextOUT->eError != PVRSRV_OK))
	{
		goto RGXCreateComputeContext_exit;
	}

	/* Lock over handle creation. */
	LockHandle(psConnection->psHandleBase);

	psRGXCreateComputeContextOUT->eError =
	    PVRSRVAllocHandleUnlocked(psConnection->psHandleBase,
				      &psRGXCreateComputeContextOUT->
				      hComputeContext,
				      (void *)psComputeContextInt,
				      PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT,
				      PVRSRV_HANDLE_ALLOC_FLAG_MULTI,
				      (PFN_HANDLE_RELEASE) &
				      PVRSRVRGXDestroyComputeContextKM);
	if (unlikely(psRGXCreateComputeContextOUT->eError != PVRSRV_OK))
	{
		UnlockHandle(psConnection->psHandleBase);
		goto RGXCreateComputeContext_exit;
	}

	/* Release now we have created handles. */
	UnlockHandle(psConnection->psHandleBase);

 RGXCreateComputeContext_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle(psConnection->psHandleBase);

	/* Unreference the previously looked up handle */
	if (hPrivDataInt)
	{
		PVRSRVReleaseHandleUnlocked(psConnection->psHandleBase,
					    hPrivData,
					    PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
	}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle(psConnection->psHandleBase);

	if (psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		if (psComputeContextInt)
		{
			PVRSRVRGXDestroyComputeContextKM(psComputeContextInt);
		}
	}

	/* Allocated space should be equal to the last updated offset */
#ifdef PVRSRV_NEED_PVR_ASSERT
	if (psRGXCreateComputeContextOUT->eError == PVRSRV_OK)
		PVR_ASSERT(ui32BufferSize == ui32NextOffset);
#endif /* PVRSRV_NEED_PVR_ASSERT */

#if defined(INTEGRITY_OS)
	if (pArrayArgsBuffer)
#else
	if (!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyComputeContext(IMG_UINT32 ui32DispatchTableEntry,
				     PVRSRV_BRIDGE_IN_RGXDESTROYCOMPUTECONTEXT *
				     psRGXDestroyComputeContextIN,
				     PVRSRV_BRIDGE_OUT_RGXDESTROYCOMPUTECONTEXT
				     * psRGXDestroyComputeContextOUT,
				     CONNECTION_DATA * psConnection)
{

	{
		PVRSRV_DEVICE_NODE *psDeviceNode = OSGetDevData(psConnection);

		/* Check that device supports the required feature */
		if ((psDeviceNode->pfnCheckDeviceFeature) &&
		    !psDeviceNode->pfnCheckDeviceFeature(psDeviceNode,
							 RGX_FEATURE_COMPUTE_BIT_MASK))
		{
			psRGXDestroyComputeContextOUT->eError =
			    PVRSRV_ERROR_NOT_SUPPORTED;

			goto RGXDestroyComputeContext_exit;
		}
	}

	/* Lock over handle destruction. */
	LockHandle(psConnection->psHandleBase);

	psRGXDestroyComputeContextOUT->eError =
	    PVRSRVDestroyHandleUnlocked(psConnection->psHandleBase,
					(IMG_HANDLE)
					psRGXDestroyComputeContextIN->
					hComputeContext,
					PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
	if (unlikely
	    ((psRGXDestroyComputeContextOUT->eError != PVRSRV_OK)
	     && (psRGXDestroyComputeContextOUT->eError != PVRSRV_ERROR_RETRY)))
	{
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVBridgeRGXDestroyComputeContext: %s",
			 PVRSRVGetErrorString(psRGXDestroyComputeContextOUT->
					      eError)));
		UnlockHandle(psConnection->psHandleBase);
		goto RGXDestroyComputeContext_exit;
	}

	/* Release now we have destroyed handles. */
	UnlockHandle(psConnection->psHandleBase);

 RGXDestroyComputeContext_exit:

	return 0;
}

#if defined(SUPPORT_SERVER_SYNC_IMPL)

static_assert(PVRSRV_MAX_SYNC_PRIMS <= IMG_UINT32_MAX,
	      "PVRSRV_MAX_SYNC_PRIMS must not be larger than IMG_UINT32_MAX");
static_assert(PVRSRV_MAX_SYNC_PRIMS <= IMG_UINT32_MAX,
	      "PVRSRV_MAX_SYNC_PRIMS must not be larger than IMG_UINT32_MAX");
static_assert(PVRSRV_MAX_SYNC_PRIMS <= IMG_UINT32_MAX,
	      "PVRSRV_MAX_SYNC_PRIMS must not be larger than IMG_UINT32_MAX");
static_assert(PVRSRV_SYNC_NAME_LENGTH <= IMG_UINT32_MAX,
	      "PVRSRV_SYNC_NAME_LENGTH must not be larger than IMG_UINT32_MAX");
static_assert(RGXFWIF_DM_INDEPENDENT_KICK_CMD_SIZE <= IMG_UINT32_MAX,
	      "RGXFWIF_DM_INDEPENDENT_KICK_CMD_SIZE must not be larger than IMG_UINT32_MAX");

static IMG_INT
PVRSRVBridgeRGXKickCDM(IMG_UINT32 ui32DispatchTableEntry,
		       PVRSRV_BRIDGE_IN_RGXKICKCDM * psRGXKickCDMIN,
		       PVRSRV_BRIDGE_OUT_RGXKICKCDM * psRGXKickCDMOUT,
		       CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hComputeContext = psRGXKickCDMIN->hComputeContext;
	RGX_SERVER_COMPUTE_CONTEXT *psComputeContextInt = NULL;
	SYNC_PRIMITIVE_BLOCK **psClientFenceUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE *hClientFenceUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 *ui32ClientFenceOffsetInt = NULL;
	IMG_UINT32 *ui32ClientFenceValueInt = NULL;
	SYNC_PRIMITIVE_BLOCK **psClientUpdateUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE *hClientUpdateUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 *ui32ClientUpdateOffsetInt = NULL;
	IMG_UINT32 *ui32ClientUpdateValueInt = NULL;
	IMG_UINT32 *ui32ServerSyncFlagsInt = NULL;
	SERVER_SYNC_PRIMITIVE **psServerSyncsInt = NULL;
	IMG_HANDLE *hServerSyncsInt2 = NULL;
	IMG_CHAR *uiUpdateFenceNameInt = NULL;
	IMG_BYTE *psDMCmdInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 0;
	IMG_UINT64 ui64BufferSize =
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ClientFenceCount *
	     sizeof(SYNC_PRIMITIVE_BLOCK *)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ClientFenceCount *
	     sizeof(IMG_HANDLE)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ClientFenceCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ClientFenceCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ClientUpdateCount *
	     sizeof(SYNC_PRIMITIVE_BLOCK *)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ClientUpdateCount *
	     sizeof(IMG_HANDLE)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ClientUpdateCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ClientUpdateCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ServerSyncCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ServerSyncCount *
	     sizeof(SERVER_SYNC_PRIMITIVE *)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32ServerSyncCount *
	     sizeof(IMG_HANDLE)) +
	    ((IMG_UINT64) PVRSRV_SYNC_NAME_LENGTH * sizeof(IMG_CHAR)) +
	    ((IMG_UINT64) psRGXKickCDMIN->ui32CmdSize * sizeof(IMG_BYTE)) + 0;

	if (unlikely
	    (psRGXKickCDMIN->ui32ClientFenceCount > PVRSRV_MAX_SYNC_PRIMS))
	{
		psRGXKickCDMOUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXKickCDM_exit;
	}

	if (unlikely
	    (psRGXKickCDMIN->ui32ClientUpdateCount > PVRSRV_MAX_SYNC_PRIMS))
	{
		psRGXKickCDMOUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXKickCDM_exit;
	}

	if (unlikely
	    (psRGXKickCDMIN->ui32ServerSyncCount > PVRSRV_MAX_SYNC_PRIMS))
	{
		psRGXKickCDMOUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXKickCDM_exit;
	}

	if (unlikely
	    (psRGXKickCDMIN->ui32CmdSize >
	     RGXFWIF_DM_INDEPENDENT_KICK_CMD_SIZE))
	{
		psRGXKickCDMOUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXKickCDM_exit;
	}

	{
		PVRSRV_DEVICE_NODE *psDeviceNode = OSGetDevData(psConnection);

		/* Check that device supports the required feature */
		if ((psDeviceNode->pfnCheckDeviceFeature) &&
		    !psDeviceNode->pfnCheckDeviceFeature(psDeviceNode,
							 RGX_FEATURE_COMPUTE_BIT_MASK))
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_NOT_SUPPORTED;

			goto RGXKickCDM_exit;
		}
	}

	if (ui64BufferSize > IMG_UINT32_MAX)
	{
		psRGXKickCDMOUT->eError = PVRSRV_ERROR_BRIDGE_BUFFER_TOO_SMALL;
		goto RGXKickCDM_exit;
	}

	ui32BufferSize = (IMG_UINT32) ui64BufferSize;

	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset =
		    PVR_ALIGN(sizeof(*psRGXKickCDMIN), sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize =
		    ui32InBufferOffset >=
		    PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 : PVRSRV_MAX_BRIDGE_IN_SIZE -
		    ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer = (IMG_BYTE *) psRGXKickCDMIN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if (!pArrayArgsBuffer)
			{
				psRGXKickCDMOUT->eError =
				    PVRSRV_ERROR_OUT_OF_MEMORY;
				goto RGXKickCDM_exit;
			}
		}
	}

	if (psRGXKickCDMIN->ui32ClientFenceCount != 0)
	{
		psClientFenceUFOSyncPrimBlockInt =
		    (SYNC_PRIMITIVE_BLOCK **) (((IMG_UINT8 *) pArrayArgsBuffer)
					       + ui32NextOffset);
		OSCachedMemSet(psClientFenceUFOSyncPrimBlockInt, 0,
			       psRGXKickCDMIN->ui32ClientFenceCount *
			       sizeof(SYNC_PRIMITIVE_BLOCK *));
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ClientFenceCount *
		    sizeof(SYNC_PRIMITIVE_BLOCK *);
		hClientFenceUFOSyncPrimBlockInt2 =
		    (IMG_HANDLE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_HANDLE);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_HANDLE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, hClientFenceUFOSyncPrimBlockInt2,
		     (const void __user *)psRGXKickCDMIN->
		     phClientFenceUFOSyncPrimBlock,
		     psRGXKickCDMIN->ui32ClientFenceCount *
		     sizeof(IMG_HANDLE)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}
	if (psRGXKickCDMIN->ui32ClientFenceCount != 0)
	{
		ui32ClientFenceOffsetInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ClientFenceOffsetInt,
		     (const void __user *)psRGXKickCDMIN->
		     pui32ClientFenceOffset,
		     psRGXKickCDMIN->ui32ClientFenceCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}
	if (psRGXKickCDMIN->ui32ClientFenceCount != 0)
	{
		ui32ClientFenceValueInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ClientFenceValueInt,
		     (const void __user *)psRGXKickCDMIN->pui32ClientFenceValue,
		     psRGXKickCDMIN->ui32ClientFenceCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}
	if (psRGXKickCDMIN->ui32ClientUpdateCount != 0)
	{
		psClientUpdateUFOSyncPrimBlockInt =
		    (SYNC_PRIMITIVE_BLOCK **) (((IMG_UINT8 *) pArrayArgsBuffer)
					       + ui32NextOffset);
		OSCachedMemSet(psClientUpdateUFOSyncPrimBlockInt, 0,
			       psRGXKickCDMIN->ui32ClientUpdateCount *
			       sizeof(SYNC_PRIMITIVE_BLOCK *));
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ClientUpdateCount *
		    sizeof(SYNC_PRIMITIVE_BLOCK *);
		hClientUpdateUFOSyncPrimBlockInt2 =
		    (IMG_HANDLE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_HANDLE);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_HANDLE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, hClientUpdateUFOSyncPrimBlockInt2,
		     (const void __user *)psRGXKickCDMIN->
		     phClientUpdateUFOSyncPrimBlock,
		     psRGXKickCDMIN->ui32ClientUpdateCount *
		     sizeof(IMG_HANDLE)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}
	if (psRGXKickCDMIN->ui32ClientUpdateCount != 0)
	{
		ui32ClientUpdateOffsetInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ClientUpdateOffsetInt,
		     (const void __user *)psRGXKickCDMIN->
		     pui32ClientUpdateOffset,
		     psRGXKickCDMIN->ui32ClientUpdateCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}
	if (psRGXKickCDMIN->ui32ClientUpdateCount != 0)
	{
		ui32ClientUpdateValueInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ClientUpdateValueInt,
		     (const void __user *)psRGXKickCDMIN->
		     pui32ClientUpdateValue,
		     psRGXKickCDMIN->ui32ClientUpdateCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}
	if (psRGXKickCDMIN->ui32ServerSyncCount != 0)
	{
		ui32ServerSyncFlagsInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ServerSyncFlagsInt,
		     (const void __user *)psRGXKickCDMIN->pui32ServerSyncFlags,
		     psRGXKickCDMIN->ui32ServerSyncCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}
	if (psRGXKickCDMIN->ui32ServerSyncCount != 0)
	{
		psServerSyncsInt =
		    (SERVER_SYNC_PRIMITIVE **) (((IMG_UINT8 *) pArrayArgsBuffer)
						+ ui32NextOffset);
		OSCachedMemSet(psServerSyncsInt, 0,
			       psRGXKickCDMIN->ui32ServerSyncCount *
			       sizeof(SERVER_SYNC_PRIMITIVE *));
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ServerSyncCount *
		    sizeof(SERVER_SYNC_PRIMITIVE *);
		hServerSyncsInt2 =
		    (IMG_HANDLE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_HANDLE);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_HANDLE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, hServerSyncsInt2,
		     (const void __user *)psRGXKickCDMIN->phServerSyncs,
		     psRGXKickCDMIN->ui32ServerSyncCount *
		     sizeof(IMG_HANDLE)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}

	{
		uiUpdateFenceNameInt =
		    (IMG_CHAR *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				  ui32NextOffset);
		ui32NextOffset += PVRSRV_SYNC_NAME_LENGTH * sizeof(IMG_CHAR);
	}

	/* Copy the data over */
	if (PVRSRV_SYNC_NAME_LENGTH * sizeof(IMG_CHAR) > 0)
	{
		if (OSCopyFromUser
		    (NULL, uiUpdateFenceNameInt,
		     (const void __user *)psRGXKickCDMIN->puiUpdateFenceName,
		     PVRSRV_SYNC_NAME_LENGTH * sizeof(IMG_CHAR)) != PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
		((IMG_CHAR *)
		 uiUpdateFenceNameInt)[(PVRSRV_SYNC_NAME_LENGTH *
					sizeof(IMG_CHAR)) - 1] = '\0';
	}
	if (psRGXKickCDMIN->ui32CmdSize != 0)
	{
		psDMCmdInt =
		    (IMG_BYTE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				  ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDMIN->ui32CmdSize * sizeof(IMG_BYTE);
	}

	/* Copy the data over */
	if (psRGXKickCDMIN->ui32CmdSize * sizeof(IMG_BYTE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, psDMCmdInt,
		     (const void __user *)psRGXKickCDMIN->psDMCmd,
		     psRGXKickCDMIN->ui32CmdSize * sizeof(IMG_BYTE)) !=
		    PVRSRV_OK)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM_exit;
		}
	}

	/* Lock over handle lookup. */
	LockHandle(psConnection->psHandleBase);

	/* Look up the address from the handle */
	psRGXKickCDMOUT->eError =
	    PVRSRVLookupHandleUnlocked(psConnection->psHandleBase,
				       (void **)&psComputeContextInt,
				       hComputeContext,
				       PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT,
				       IMG_TRUE);
	if (unlikely(psRGXKickCDMOUT->eError != PVRSRV_OK))
	{
		UnlockHandle(psConnection->psHandleBase);
		goto RGXKickCDM_exit;
	}

	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDMIN->ui32ClientFenceCount; i++)
		{
			/* Look up the address from the handle */
			psRGXKickCDMOUT->eError =
			    PVRSRVLookupHandleUnlocked(psConnection->
						       psHandleBase,
						       (void **)
						       &psClientFenceUFOSyncPrimBlockInt
						       [i],
						       hClientFenceUFOSyncPrimBlockInt2
						       [i],
						       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK,
						       IMG_TRUE);
			if (unlikely(psRGXKickCDMOUT->eError != PVRSRV_OK))
			{
				UnlockHandle(psConnection->psHandleBase);
				goto RGXKickCDM_exit;
			}
		}
	}

	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDMIN->ui32ClientUpdateCount; i++)
		{
			/* Look up the address from the handle */
			psRGXKickCDMOUT->eError =
			    PVRSRVLookupHandleUnlocked(psConnection->
						       psHandleBase,
						       (void **)
						       &psClientUpdateUFOSyncPrimBlockInt
						       [i],
						       hClientUpdateUFOSyncPrimBlockInt2
						       [i],
						       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK,
						       IMG_TRUE);
			if (unlikely(psRGXKickCDMOUT->eError != PVRSRV_OK))
			{
				UnlockHandle(psConnection->psHandleBase);
				goto RGXKickCDM_exit;
			}
		}
	}

	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDMIN->ui32ServerSyncCount; i++)
		{
			/* Look up the address from the handle */
			psRGXKickCDMOUT->eError =
			    PVRSRVLookupHandleUnlocked(psConnection->
						       psHandleBase,
						       (void **)
						       &psServerSyncsInt[i],
						       hServerSyncsInt2[i],
						       PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE,
						       IMG_TRUE);
			if (unlikely(psRGXKickCDMOUT->eError != PVRSRV_OK))
			{
				UnlockHandle(psConnection->psHandleBase);
				goto RGXKickCDM_exit;
			}
		}
	}
	/* Release now we have looked up handles. */
	UnlockHandle(psConnection->psHandleBase);

	psRGXKickCDMOUT->eError =
	    PVRSRVRGXKickCDMKM(psComputeContextInt,
			       psRGXKickCDMIN->ui32ClientCacheOpSeqNum,
			       psRGXKickCDMIN->ui32ClientFenceCount,
			       psClientFenceUFOSyncPrimBlockInt,
			       ui32ClientFenceOffsetInt,
			       ui32ClientFenceValueInt,
			       psRGXKickCDMIN->ui32ClientUpdateCount,
			       psClientUpdateUFOSyncPrimBlockInt,
			       ui32ClientUpdateOffsetInt,
			       ui32ClientUpdateValueInt,
			       psRGXKickCDMIN->ui32ServerSyncCount,
			       ui32ServerSyncFlagsInt,
			       psServerSyncsInt,
			       psRGXKickCDMIN->hCheckFenceFd,
			       psRGXKickCDMIN->hUpdateTimeline,
			       &psRGXKickCDMOUT->hUpdateFence,
			       uiUpdateFenceNameInt,
			       psRGXKickCDMIN->ui32CmdSize,
			       psDMCmdInt,
			       psRGXKickCDMIN->ui32PDumpFlags,
			       psRGXKickCDMIN->ui32ExtJobRef);

 RGXKickCDM_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle(psConnection->psHandleBase);

	/* Unreference the previously looked up handle */
	if (psComputeContextInt)
	{
		PVRSRVReleaseHandleUnlocked(psConnection->psHandleBase,
					    hComputeContext,
					    PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
	}

	if (hClientFenceUFOSyncPrimBlockInt2)
	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDMIN->ui32ClientFenceCount; i++)
		{

			/* Unreference the previously looked up handle */
			if (psClientFenceUFOSyncPrimBlockInt[i])
			{
				PVRSRVReleaseHandleUnlocked(psConnection->
							    psHandleBase,
							    hClientFenceUFOSyncPrimBlockInt2
							    [i],
							    PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
			}
		}
	}

	if (hClientUpdateUFOSyncPrimBlockInt2)
	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDMIN->ui32ClientUpdateCount; i++)
		{

			/* Unreference the previously looked up handle */
			if (psClientUpdateUFOSyncPrimBlockInt[i])
			{
				PVRSRVReleaseHandleUnlocked(psConnection->
							    psHandleBase,
							    hClientUpdateUFOSyncPrimBlockInt2
							    [i],
							    PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
			}
		}
	}

	if (hServerSyncsInt2)
	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDMIN->ui32ServerSyncCount; i++)
		{

			/* Unreference the previously looked up handle */
			if (psServerSyncsInt[i])
			{
				PVRSRVReleaseHandleUnlocked(psConnection->
							    psHandleBase,
							    hServerSyncsInt2[i],
							    PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
			}
		}
	}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle(psConnection->psHandleBase);

	/* Allocated space should be equal to the last updated offset */
#ifdef PVRSRV_NEED_PVR_ASSERT
	if (psRGXKickCDMOUT->eError == PVRSRV_OK)
		PVR_ASSERT(ui32BufferSize == ui32NextOffset);
#endif /* PVRSRV_NEED_PVR_ASSERT */

#if defined(INTEGRITY_OS)
	if (pArrayArgsBuffer)
#else
	if (!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);

	return 0;
}

#else
#define PVRSRVBridgeRGXKickCDM NULL
#endif

static IMG_INT
PVRSRVBridgeRGXFlushComputeData(IMG_UINT32 ui32DispatchTableEntry,
				PVRSRV_BRIDGE_IN_RGXFLUSHCOMPUTEDATA *
				psRGXFlushComputeDataIN,
				PVRSRV_BRIDGE_OUT_RGXFLUSHCOMPUTEDATA *
				psRGXFlushComputeDataOUT,
				CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hComputeContext = psRGXFlushComputeDataIN->hComputeContext;
	RGX_SERVER_COMPUTE_CONTEXT *psComputeContextInt = NULL;

	{
		PVRSRV_DEVICE_NODE *psDeviceNode = OSGetDevData(psConnection);

		/* Check that device supports the required feature */
		if ((psDeviceNode->pfnCheckDeviceFeature) &&
		    !psDeviceNode->pfnCheckDeviceFeature(psDeviceNode,
							 RGX_FEATURE_COMPUTE_BIT_MASK))
		{
			psRGXFlushComputeDataOUT->eError =
			    PVRSRV_ERROR_NOT_SUPPORTED;

			goto RGXFlushComputeData_exit;
		}
	}

	/* Lock over handle lookup. */
	LockHandle(psConnection->psHandleBase);

	/* Look up the address from the handle */
	psRGXFlushComputeDataOUT->eError =
	    PVRSRVLookupHandleUnlocked(psConnection->psHandleBase,
				       (void **)&psComputeContextInt,
				       hComputeContext,
				       PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT,
				       IMG_TRUE);
	if (unlikely(psRGXFlushComputeDataOUT->eError != PVRSRV_OK))
	{
		UnlockHandle(psConnection->psHandleBase);
		goto RGXFlushComputeData_exit;
	}
	/* Release now we have looked up handles. */
	UnlockHandle(psConnection->psHandleBase);

	psRGXFlushComputeDataOUT->eError =
	    PVRSRVRGXFlushComputeDataKM(psComputeContextInt);

 RGXFlushComputeData_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle(psConnection->psHandleBase);

	/* Unreference the previously looked up handle */
	if (psComputeContextInt)
	{
		PVRSRVReleaseHandleUnlocked(psConnection->psHandleBase,
					    hComputeContext,
					    PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
	}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle(psConnection->psHandleBase);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXSetComputeContextPriority(IMG_UINT32 ui32DispatchTableEntry,
					 PVRSRV_BRIDGE_IN_RGXSETCOMPUTECONTEXTPRIORITY
					 * psRGXSetComputeContextPriorityIN,
					 PVRSRV_BRIDGE_OUT_RGXSETCOMPUTECONTEXTPRIORITY
					 * psRGXSetComputeContextPriorityOUT,
					 CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hComputeContext =
	    psRGXSetComputeContextPriorityIN->hComputeContext;
	RGX_SERVER_COMPUTE_CONTEXT *psComputeContextInt = NULL;

	{
		PVRSRV_DEVICE_NODE *psDeviceNode = OSGetDevData(psConnection);

		/* Check that device supports the required feature */
		if ((psDeviceNode->pfnCheckDeviceFeature) &&
		    !psDeviceNode->pfnCheckDeviceFeature(psDeviceNode,
							 RGX_FEATURE_COMPUTE_BIT_MASK))
		{
			psRGXSetComputeContextPriorityOUT->eError =
			    PVRSRV_ERROR_NOT_SUPPORTED;

			goto RGXSetComputeContextPriority_exit;
		}
	}

	/* Lock over handle lookup. */
	LockHandle(psConnection->psHandleBase);

	/* Look up the address from the handle */
	psRGXSetComputeContextPriorityOUT->eError =
	    PVRSRVLookupHandleUnlocked(psConnection->psHandleBase,
				       (void **)&psComputeContextInt,
				       hComputeContext,
				       PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT,
				       IMG_TRUE);
	if (unlikely(psRGXSetComputeContextPriorityOUT->eError != PVRSRV_OK))
	{
		UnlockHandle(psConnection->psHandleBase);
		goto RGXSetComputeContextPriority_exit;
	}
	/* Release now we have looked up handles. */
	UnlockHandle(psConnection->psHandleBase);

	psRGXSetComputeContextPriorityOUT->eError =
	    PVRSRVRGXSetComputeContextPriorityKM(psConnection,
						 OSGetDevData(psConnection),
						 psComputeContextInt,
						 psRGXSetComputeContextPriorityIN->
						 ui32Priority);

 RGXSetComputeContextPriority_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle(psConnection->psHandleBase);

	/* Unreference the previously looked up handle */
	if (psComputeContextInt)
	{
		PVRSRVReleaseHandleUnlocked(psConnection->psHandleBase,
					    hComputeContext,
					    PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
	}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle(psConnection->psHandleBase);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXGetLastComputeContextResetReason(IMG_UINT32
						ui32DispatchTableEntry,
						PVRSRV_BRIDGE_IN_RGXGETLASTCOMPUTECONTEXTRESETREASON
						*
						psRGXGetLastComputeContextResetReasonIN,
						PVRSRV_BRIDGE_OUT_RGXGETLASTCOMPUTECONTEXTRESETREASON
						*
						psRGXGetLastComputeContextResetReasonOUT,
						CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hComputeContext =
	    psRGXGetLastComputeContextResetReasonIN->hComputeContext;
	RGX_SERVER_COMPUTE_CONTEXT *psComputeContextInt = NULL;

	{
		PVRSRV_DEVICE_NODE *psDeviceNode = OSGetDevData(psConnection);

		/* Check that device supports the required feature */
		if ((psDeviceNode->pfnCheckDeviceFeature) &&
		    !psDeviceNode->pfnCheckDeviceFeature(psDeviceNode,
							 RGX_FEATURE_COMPUTE_BIT_MASK))
		{
			psRGXGetLastComputeContextResetReasonOUT->eError =
			    PVRSRV_ERROR_NOT_SUPPORTED;

			goto RGXGetLastComputeContextResetReason_exit;
		}
	}

	/* Lock over handle lookup. */
	LockHandle(psConnection->psHandleBase);

	/* Look up the address from the handle */
	psRGXGetLastComputeContextResetReasonOUT->eError =
	    PVRSRVLookupHandleUnlocked(psConnection->psHandleBase,
				       (void **)&psComputeContextInt,
				       hComputeContext,
				       PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT,
				       IMG_TRUE);
	if (unlikely
	    (psRGXGetLastComputeContextResetReasonOUT->eError != PVRSRV_OK))
	{
		UnlockHandle(psConnection->psHandleBase);
		goto RGXGetLastComputeContextResetReason_exit;
	}
	/* Release now we have looked up handles. */
	UnlockHandle(psConnection->psHandleBase);

	psRGXGetLastComputeContextResetReasonOUT->eError =
	    PVRSRVRGXGetLastComputeContextResetReasonKM(psComputeContextInt,
							&psRGXGetLastComputeContextResetReasonOUT->
							ui32LastResetReason,
							&psRGXGetLastComputeContextResetReasonOUT->
							ui32LastResetJobRef);

 RGXGetLastComputeContextResetReason_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle(psConnection->psHandleBase);

	/* Unreference the previously looked up handle */
	if (psComputeContextInt)
	{
		PVRSRVReleaseHandleUnlocked(psConnection->psHandleBase,
					    hComputeContext,
					    PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
	}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle(psConnection->psHandleBase);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXNotifyComputeWriteOffsetUpdate(IMG_UINT32 ui32DispatchTableEntry,
					      PVRSRV_BRIDGE_IN_RGXNOTIFYCOMPUTEWRITEOFFSETUPDATE
					      *
					      psRGXNotifyComputeWriteOffsetUpdateIN,
					      PVRSRV_BRIDGE_OUT_RGXNOTIFYCOMPUTEWRITEOFFSETUPDATE
					      *
					      psRGXNotifyComputeWriteOffsetUpdateOUT,
					      CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hComputeContext =
	    psRGXNotifyComputeWriteOffsetUpdateIN->hComputeContext;
	RGX_SERVER_COMPUTE_CONTEXT *psComputeContextInt = NULL;

	{
		PVRSRV_DEVICE_NODE *psDeviceNode = OSGetDevData(psConnection);

		/* Check that device supports the required feature */
		if ((psDeviceNode->pfnCheckDeviceFeature) &&
		    !psDeviceNode->pfnCheckDeviceFeature(psDeviceNode,
							 RGX_FEATURE_COMPUTE_BIT_MASK))
		{
			psRGXNotifyComputeWriteOffsetUpdateOUT->eError =
			    PVRSRV_ERROR_NOT_SUPPORTED;

			goto RGXNotifyComputeWriteOffsetUpdate_exit;
		}
	}

	/* Lock over handle lookup. */
	LockHandle(psConnection->psHandleBase);

	/* Look up the address from the handle */
	psRGXNotifyComputeWriteOffsetUpdateOUT->eError =
	    PVRSRVLookupHandleUnlocked(psConnection->psHandleBase,
				       (void **)&psComputeContextInt,
				       hComputeContext,
				       PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT,
				       IMG_TRUE);
	if (unlikely
	    (psRGXNotifyComputeWriteOffsetUpdateOUT->eError != PVRSRV_OK))
	{
		UnlockHandle(psConnection->psHandleBase);
		goto RGXNotifyComputeWriteOffsetUpdate_exit;
	}
	/* Release now we have looked up handles. */
	UnlockHandle(psConnection->psHandleBase);

	psRGXNotifyComputeWriteOffsetUpdateOUT->eError =
	    PVRSRVRGXNotifyComputeWriteOffsetUpdateKM(psComputeContextInt);

 RGXNotifyComputeWriteOffsetUpdate_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle(psConnection->psHandleBase);

	/* Unreference the previously looked up handle */
	if (psComputeContextInt)
	{
		PVRSRVReleaseHandleUnlocked(psConnection->psHandleBase,
					    hComputeContext,
					    PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
	}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle(psConnection->psHandleBase);

	return 0;
}

#if !defined(SUPPORT_SERVER_SYNC_IMPL)

static_assert(PVRSRV_MAX_SYNC_PRIMS <= IMG_UINT32_MAX,
	      "PVRSRV_MAX_SYNC_PRIMS must not be larger than IMG_UINT32_MAX");
static_assert(PVRSRV_MAX_SYNC_PRIMS <= IMG_UINT32_MAX,
	      "PVRSRV_MAX_SYNC_PRIMS must not be larger than IMG_UINT32_MAX");
static_assert(PVRSRV_SYNC_NAME_LENGTH <= IMG_UINT32_MAX,
	      "PVRSRV_SYNC_NAME_LENGTH must not be larger than IMG_UINT32_MAX");
static_assert(RGXFWIF_DM_INDEPENDENT_KICK_CMD_SIZE <= IMG_UINT32_MAX,
	      "RGXFWIF_DM_INDEPENDENT_KICK_CMD_SIZE must not be larger than IMG_UINT32_MAX");

static IMG_INT
PVRSRVBridgeRGXKickCDM2(IMG_UINT32 ui32DispatchTableEntry,
			PVRSRV_BRIDGE_IN_RGXKICKCDM2 * psRGXKickCDM2IN,
			PVRSRV_BRIDGE_OUT_RGXKICKCDM2 * psRGXKickCDM2OUT,
			CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hComputeContext = psRGXKickCDM2IN->hComputeContext;
	RGX_SERVER_COMPUTE_CONTEXT *psComputeContextInt = NULL;
	SYNC_PRIMITIVE_BLOCK **psClientFenceUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE *hClientFenceUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 *ui32ClientFenceOffsetInt = NULL;
	IMG_UINT32 *ui32ClientFenceValueInt = NULL;
	SYNC_PRIMITIVE_BLOCK **psClientUpdateUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE *hClientUpdateUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 *ui32ClientUpdateOffsetInt = NULL;
	IMG_UINT32 *ui32ClientUpdateValueInt = NULL;
	IMG_CHAR *uiUpdateFenceNameInt = NULL;
	IMG_BYTE *psDMCmdInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 0;
	IMG_UINT64 ui64BufferSize =
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32ClientFenceCount *
	     sizeof(SYNC_PRIMITIVE_BLOCK *)) +
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32ClientFenceCount *
	     sizeof(IMG_HANDLE)) +
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32ClientFenceCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32ClientFenceCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32ClientUpdateCount *
	     sizeof(SYNC_PRIMITIVE_BLOCK *)) +
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32ClientUpdateCount *
	     sizeof(IMG_HANDLE)) +
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32ClientUpdateCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32ClientUpdateCount *
	     sizeof(IMG_UINT32)) +
	    ((IMG_UINT64) PVRSRV_SYNC_NAME_LENGTH * sizeof(IMG_CHAR)) +
	    ((IMG_UINT64) psRGXKickCDM2IN->ui32CmdSize * sizeof(IMG_BYTE)) + 0;

	if (unlikely
	    (psRGXKickCDM2IN->ui32ClientFenceCount > PVRSRV_MAX_SYNC_PRIMS))
	{
		psRGXKickCDM2OUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXKickCDM2_exit;
	}

	if (unlikely
	    (psRGXKickCDM2IN->ui32ClientUpdateCount > PVRSRV_MAX_SYNC_PRIMS))
	{
		psRGXKickCDM2OUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXKickCDM2_exit;
	}

	if (unlikely
	    (psRGXKickCDM2IN->ui32CmdSize >
	     RGXFWIF_DM_INDEPENDENT_KICK_CMD_SIZE))
	{
		psRGXKickCDM2OUT->eError =
		    PVRSRV_ERROR_BRIDGE_ARRAY_SIZE_TOO_BIG;
		goto RGXKickCDM2_exit;
	}

	{
		PVRSRV_DEVICE_NODE *psDeviceNode = OSGetDevData(psConnection);

		/* Check that device supports the required feature */
		if ((psDeviceNode->pfnCheckDeviceFeature) &&
		    !psDeviceNode->pfnCheckDeviceFeature(psDeviceNode,
							 RGX_FEATURE_COMPUTE_BIT_MASK))
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_NOT_SUPPORTED;

			goto RGXKickCDM2_exit;
		}
	}

	if (ui64BufferSize > IMG_UINT32_MAX)
	{
		psRGXKickCDM2OUT->eError = PVRSRV_ERROR_BRIDGE_BUFFER_TOO_SMALL;
		goto RGXKickCDM2_exit;
	}

	ui32BufferSize = (IMG_UINT32) ui64BufferSize;

	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset =
		    PVR_ALIGN(sizeof(*psRGXKickCDM2IN), sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize =
		    ui32InBufferOffset >=
		    PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 : PVRSRV_MAX_BRIDGE_IN_SIZE -
		    ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer = (IMG_BYTE *) psRGXKickCDM2IN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if (!pArrayArgsBuffer)
			{
				psRGXKickCDM2OUT->eError =
				    PVRSRV_ERROR_OUT_OF_MEMORY;
				goto RGXKickCDM2_exit;
			}
		}
	}

	if (psRGXKickCDM2IN->ui32ClientFenceCount != 0)
	{
		psClientFenceUFOSyncPrimBlockInt =
		    (SYNC_PRIMITIVE_BLOCK **) (((IMG_UINT8 *) pArrayArgsBuffer)
					       + ui32NextOffset);
		OSCachedMemSet(psClientFenceUFOSyncPrimBlockInt, 0,
			       psRGXKickCDM2IN->ui32ClientFenceCount *
			       sizeof(SYNC_PRIMITIVE_BLOCK *));
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32ClientFenceCount *
		    sizeof(SYNC_PRIMITIVE_BLOCK *);
		hClientFenceUFOSyncPrimBlockInt2 =
		    (IMG_HANDLE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32ClientFenceCount * sizeof(IMG_HANDLE);
	}

	/* Copy the data over */
	if (psRGXKickCDM2IN->ui32ClientFenceCount * sizeof(IMG_HANDLE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, hClientFenceUFOSyncPrimBlockInt2,
		     (const void __user *)psRGXKickCDM2IN->
		     phClientFenceUFOSyncPrimBlock,
		     psRGXKickCDM2IN->ui32ClientFenceCount *
		     sizeof(IMG_HANDLE)) != PVRSRV_OK)
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM2_exit;
		}
	}
	if (psRGXKickCDM2IN->ui32ClientFenceCount != 0)
	{
		ui32ClientFenceOffsetInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32ClientFenceCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDM2IN->ui32ClientFenceCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ClientFenceOffsetInt,
		     (const void __user *)psRGXKickCDM2IN->
		     pui32ClientFenceOffset,
		     psRGXKickCDM2IN->ui32ClientFenceCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM2_exit;
		}
	}
	if (psRGXKickCDM2IN->ui32ClientFenceCount != 0)
	{
		ui32ClientFenceValueInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32ClientFenceCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDM2IN->ui32ClientFenceCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ClientFenceValueInt,
		     (const void __user *)psRGXKickCDM2IN->
		     pui32ClientFenceValue,
		     psRGXKickCDM2IN->ui32ClientFenceCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM2_exit;
		}
	}
	if (psRGXKickCDM2IN->ui32ClientUpdateCount != 0)
	{
		psClientUpdateUFOSyncPrimBlockInt =
		    (SYNC_PRIMITIVE_BLOCK **) (((IMG_UINT8 *) pArrayArgsBuffer)
					       + ui32NextOffset);
		OSCachedMemSet(psClientUpdateUFOSyncPrimBlockInt, 0,
			       psRGXKickCDM2IN->ui32ClientUpdateCount *
			       sizeof(SYNC_PRIMITIVE_BLOCK *));
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32ClientUpdateCount *
		    sizeof(SYNC_PRIMITIVE_BLOCK *);
		hClientUpdateUFOSyncPrimBlockInt2 =
		    (IMG_HANDLE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32ClientUpdateCount * sizeof(IMG_HANDLE);
	}

	/* Copy the data over */
	if (psRGXKickCDM2IN->ui32ClientUpdateCount * sizeof(IMG_HANDLE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, hClientUpdateUFOSyncPrimBlockInt2,
		     (const void __user *)psRGXKickCDM2IN->
		     phClientUpdateUFOSyncPrimBlock,
		     psRGXKickCDM2IN->ui32ClientUpdateCount *
		     sizeof(IMG_HANDLE)) != PVRSRV_OK)
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM2_exit;
		}
	}
	if (psRGXKickCDM2IN->ui32ClientUpdateCount != 0)
	{
		ui32ClientUpdateOffsetInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32ClientUpdateCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDM2IN->ui32ClientUpdateCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ClientUpdateOffsetInt,
		     (const void __user *)psRGXKickCDM2IN->
		     pui32ClientUpdateOffset,
		     psRGXKickCDM2IN->ui32ClientUpdateCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM2_exit;
		}
	}
	if (psRGXKickCDM2IN->ui32ClientUpdateCount != 0)
	{
		ui32ClientUpdateValueInt =
		    (IMG_UINT32 *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				    ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32ClientUpdateCount * sizeof(IMG_UINT32);
	}

	/* Copy the data over */
	if (psRGXKickCDM2IN->ui32ClientUpdateCount * sizeof(IMG_UINT32) > 0)
	{
		if (OSCopyFromUser
		    (NULL, ui32ClientUpdateValueInt,
		     (const void __user *)psRGXKickCDM2IN->
		     pui32ClientUpdateValue,
		     psRGXKickCDM2IN->ui32ClientUpdateCount *
		     sizeof(IMG_UINT32)) != PVRSRV_OK)
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM2_exit;
		}
	}

	{
		uiUpdateFenceNameInt =
		    (IMG_CHAR *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				  ui32NextOffset);
		ui32NextOffset += PVRSRV_SYNC_NAME_LENGTH * sizeof(IMG_CHAR);
	}

	/* Copy the data over */
	if (PVRSRV_SYNC_NAME_LENGTH * sizeof(IMG_CHAR) > 0)
	{
		if (OSCopyFromUser
		    (NULL, uiUpdateFenceNameInt,
		     (const void __user *)psRGXKickCDM2IN->puiUpdateFenceName,
		     PVRSRV_SYNC_NAME_LENGTH * sizeof(IMG_CHAR)) != PVRSRV_OK)
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM2_exit;
		}
		((IMG_CHAR *)
		 uiUpdateFenceNameInt)[(PVRSRV_SYNC_NAME_LENGTH *
					sizeof(IMG_CHAR)) - 1] = '\0';
	}
	if (psRGXKickCDM2IN->ui32CmdSize != 0)
	{
		psDMCmdInt =
		    (IMG_BYTE *) (((IMG_UINT8 *) pArrayArgsBuffer) +
				  ui32NextOffset);
		ui32NextOffset +=
		    psRGXKickCDM2IN->ui32CmdSize * sizeof(IMG_BYTE);
	}

	/* Copy the data over */
	if (psRGXKickCDM2IN->ui32CmdSize * sizeof(IMG_BYTE) > 0)
	{
		if (OSCopyFromUser
		    (NULL, psDMCmdInt,
		     (const void __user *)psRGXKickCDM2IN->psDMCmd,
		     psRGXKickCDM2IN->ui32CmdSize * sizeof(IMG_BYTE)) !=
		    PVRSRV_OK)
		{
			psRGXKickCDM2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

			goto RGXKickCDM2_exit;
		}
	}

	/* Lock over handle lookup. */
	LockHandle(psConnection->psHandleBase);

	/* Look up the address from the handle */
	psRGXKickCDM2OUT->eError =
	    PVRSRVLookupHandleUnlocked(psConnection->psHandleBase,
				       (void **)&psComputeContextInt,
				       hComputeContext,
				       PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT,
				       IMG_TRUE);
	if (unlikely(psRGXKickCDM2OUT->eError != PVRSRV_OK))
	{
		UnlockHandle(psConnection->psHandleBase);
		goto RGXKickCDM2_exit;
	}

	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDM2IN->ui32ClientFenceCount; i++)
		{
			/* Look up the address from the handle */
			psRGXKickCDM2OUT->eError =
			    PVRSRVLookupHandleUnlocked(psConnection->
						       psHandleBase,
						       (void **)
						       &psClientFenceUFOSyncPrimBlockInt
						       [i],
						       hClientFenceUFOSyncPrimBlockInt2
						       [i],
						       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK,
						       IMG_TRUE);
			if (unlikely(psRGXKickCDM2OUT->eError != PVRSRV_OK))
			{
				UnlockHandle(psConnection->psHandleBase);
				goto RGXKickCDM2_exit;
			}
		}
	}

	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDM2IN->ui32ClientUpdateCount; i++)
		{
			/* Look up the address from the handle */
			psRGXKickCDM2OUT->eError =
			    PVRSRVLookupHandleUnlocked(psConnection->
						       psHandleBase,
						       (void **)
						       &psClientUpdateUFOSyncPrimBlockInt
						       [i],
						       hClientUpdateUFOSyncPrimBlockInt2
						       [i],
						       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK,
						       IMG_TRUE);
			if (unlikely(psRGXKickCDM2OUT->eError != PVRSRV_OK))
			{
				UnlockHandle(psConnection->psHandleBase);
				goto RGXKickCDM2_exit;
			}
		}
	}
	/* Release now we have looked up handles. */
	UnlockHandle(psConnection->psHandleBase);

	psRGXKickCDM2OUT->eError =
	    PVRSRVRGXKickCDM2KM(psComputeContextInt,
				psRGXKickCDM2IN->ui32ClientCacheOpSeqNum,
				psRGXKickCDM2IN->ui32ClientFenceCount,
				psClientFenceUFOSyncPrimBlockInt,
				ui32ClientFenceOffsetInt,
				ui32ClientFenceValueInt,
				psRGXKickCDM2IN->ui32ClientUpdateCount,
				psClientUpdateUFOSyncPrimBlockInt,
				ui32ClientUpdateOffsetInt,
				ui32ClientUpdateValueInt,
				psRGXKickCDM2IN->hCheckFenceFd,
				psRGXKickCDM2IN->hUpdateTimeline,
				&psRGXKickCDM2OUT->hUpdateFence,
				uiUpdateFenceNameInt,
				psRGXKickCDM2IN->ui32CmdSize,
				psDMCmdInt,
				psRGXKickCDM2IN->ui32PDumpFlags,
				psRGXKickCDM2IN->ui32ExtJobRef);

 RGXKickCDM2_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle(psConnection->psHandleBase);

	/* Unreference the previously looked up handle */
	if (psComputeContextInt)
	{
		PVRSRVReleaseHandleUnlocked(psConnection->psHandleBase,
					    hComputeContext,
					    PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
	}

	if (hClientFenceUFOSyncPrimBlockInt2)
	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDM2IN->ui32ClientFenceCount; i++)
		{

			/* Unreference the previously looked up handle */
			if (psClientFenceUFOSyncPrimBlockInt[i])
			{
				PVRSRVReleaseHandleUnlocked(psConnection->
							    psHandleBase,
							    hClientFenceUFOSyncPrimBlockInt2
							    [i],
							    PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
			}
		}
	}

	if (hClientUpdateUFOSyncPrimBlockInt2)
	{
		IMG_UINT32 i;

		for (i = 0; i < psRGXKickCDM2IN->ui32ClientUpdateCount; i++)
		{

			/* Unreference the previously looked up handle */
			if (psClientUpdateUFOSyncPrimBlockInt[i])
			{
				PVRSRVReleaseHandleUnlocked(psConnection->
							    psHandleBase,
							    hClientUpdateUFOSyncPrimBlockInt2
							    [i],
							    PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
			}
		}
	}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle(psConnection->psHandleBase);

	/* Allocated space should be equal to the last updated offset */
#ifdef PVRSRV_NEED_PVR_ASSERT
	if (psRGXKickCDM2OUT->eError == PVRSRV_OK)
		PVR_ASSERT(ui32BufferSize == ui32NextOffset);
#endif /* PVRSRV_NEED_PVR_ASSERT */

#if defined(INTEGRITY_OS)
	if (pArrayArgsBuffer)
#else
	if (!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);

	return 0;
}

#else
#define PVRSRVBridgeRGXKickCDM2 NULL
#endif

/* ***************************************************************************
 * Server bridge dispatch related glue
 */

static IMG_BOOL bUseLock = IMG_TRUE;

PVRSRV_ERROR InitRGXCMPBridge(void);
PVRSRV_ERROR DeinitRGXCMPBridge(void);

/*
 * Register all RGXCMP functions with services
 */
PVRSRV_ERROR InitRGXCMPBridge(void)
{

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
			      PVRSRV_BRIDGE_RGXCMP_RGXCREATECOMPUTECONTEXT,
			      PVRSRVBridgeRGXCreateComputeContext, NULL,
			      bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
			      PVRSRV_BRIDGE_RGXCMP_RGXDESTROYCOMPUTECONTEXT,
			      PVRSRVBridgeRGXDestroyComputeContext, NULL,
			      bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
			      PVRSRV_BRIDGE_RGXCMP_RGXKICKCDM,
			      PVRSRVBridgeRGXKickCDM, NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
			      PVRSRV_BRIDGE_RGXCMP_RGXFLUSHCOMPUTEDATA,
			      PVRSRVBridgeRGXFlushComputeData, NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
			      PVRSRV_BRIDGE_RGXCMP_RGXSETCOMPUTECONTEXTPRIORITY,
			      PVRSRVBridgeRGXSetComputeContextPriority, NULL,
			      bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
			      PVRSRV_BRIDGE_RGXCMP_RGXGETLASTCOMPUTECONTEXTRESETREASON,
			      PVRSRVBridgeRGXGetLastComputeContextResetReason,
			      NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
			      PVRSRV_BRIDGE_RGXCMP_RGXNOTIFYCOMPUTEWRITEOFFSETUPDATE,
			      PVRSRVBridgeRGXNotifyComputeWriteOffsetUpdate,
			      NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
			      PVRSRV_BRIDGE_RGXCMP_RGXKICKCDM2,
			      PVRSRVBridgeRGXKickCDM2, NULL, bUseLock);

	return PVRSRV_OK;
}

/*
 * Unregister all rgxcmp functions with services
 */
PVRSRV_ERROR DeinitRGXCMPBridge(void)
{

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
				PVRSRV_BRIDGE_RGXCMP_RGXCREATECOMPUTECONTEXT);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
				PVRSRV_BRIDGE_RGXCMP_RGXDESTROYCOMPUTECONTEXT);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
				PVRSRV_BRIDGE_RGXCMP_RGXKICKCDM);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
				PVRSRV_BRIDGE_RGXCMP_RGXFLUSHCOMPUTEDATA);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
				PVRSRV_BRIDGE_RGXCMP_RGXSETCOMPUTECONTEXTPRIORITY);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
				PVRSRV_BRIDGE_RGXCMP_RGXGETLASTCOMPUTECONTEXTRESETREASON);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
				PVRSRV_BRIDGE_RGXCMP_RGXNOTIFYCOMPUTEWRITEOFFSETUPDATE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP,
				PVRSRV_BRIDGE_RGXCMP_RGXKICKCDM2);

	return PVRSRV_OK;
}
