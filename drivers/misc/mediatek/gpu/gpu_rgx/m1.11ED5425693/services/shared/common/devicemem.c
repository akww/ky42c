/*************************************************************************/ /*!
@File
@Title          Device Memory Management
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Front End (nominally Client side part, but now invokable
                from server too) of device memory management
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
 */ /**************************************************************************/

#include "devicemem.h"
#include "img_types.h"
#include "img_defs.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"
#include "allocmem.h"
#include "ra.h"
#include "osfunc.h"
#include "osmmap.h"
#include "devicemem_utils.h"
#include "client_mm_bridge.h"
#include "client_cache_bridge.h"
#include "services_km.h"

#if defined(PDUMP)
#if defined(__KERNEL__)
#include "pdump_km.h"
#else
#include "pdump_um.h"
#endif
#include "devicemem_pdump.h"
#endif
#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
#include "client_ri_bridge.h"
#endif
#include "client_devicememhistory_bridge.h"
#include "info_page_client.h"

#include "rgx_heaps.h"
#if defined(__KERNEL__)
#include "srvcore.h"
#include "pvrsrv.h"
#include "rgxdefs_km.h"
#include "rgx_bvnc_defs_km.h"
#include "device.h"
#include "rgxdevice.h"
#include "pvr_ricommon.h"
#if defined(LINUX)
#include "linux/kernel.h"
#endif
#else
#include "srvcore_intern.h"
#include "rgxdefs.h"
#endif

#if defined(__KERNEL__) && defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
extern PVRSRV_ERROR RIDumpAllKM(void);
#endif

#if defined(__KERNEL__)
#define GET_ERROR_STRING(eError) PVRSRVGetErrorString(eError)
#else
#define GET_ERROR_STRING(eError) PVRSRVGetErrorString(eError)
#endif
/*****************************************************************************
 *                    Sub allocation internals                               *
 *****************************************************************************/

static INLINE void
_CheckAnnotationLength(const IMG_CHAR *pszAnnotation)
{
	IMG_UINT32 length = OSStringLength(pszAnnotation);

	if (length >= DEVMEM_ANNOTATION_MAX_LEN)
	{
		PVR_DPF((PVR_DBG_WARNING, "%s: Annotation \"%s\" has been truncated to %d characters from %d characters",
				__func__, pszAnnotation, DEVMEM_ANNOTATION_MAX_LEN - 1, length));
	}
}

static PVRSRV_ERROR
_AllocateDeviceMemory(SHARED_DEV_CONNECTION hDevConnection,
		IMG_UINT32 uiLog2Quantum,
		IMG_DEVMEM_SIZE_T uiSize,
		IMG_DEVMEM_SIZE_T uiChunkSize,
		IMG_UINT32 ui32NumPhysChunks,
		IMG_UINT32 ui32NumVirtChunks,
		IMG_UINT32 *pui32MappingTable,
		IMG_DEVMEM_ALIGN_T uiAlign,
		DEVMEM_FLAGS_T uiFlags,
		IMG_BOOL bExportable,
		const IMG_CHAR *pszAnnotation,
		DEVMEM_IMPORT **ppsImport)
{
	DEVMEM_IMPORT *psImport;
	DEVMEM_FLAGS_T uiPMRFlags;
	IMG_HANDLE hPMR;
	PVRSRV_ERROR eError;

	eError = _DevmemImportStructAlloc(hDevConnection,
			&psImport);
	if (eError != PVRSRV_OK)
	{
		goto failAlloc;
	}

	/* check if shift value is not too big (sizeof(1ULL)) */
	PVR_ASSERT(uiLog2Quantum < sizeof(unsigned long long) * 8);
	/* Check the size is a multiple of the quantum */
	PVR_ASSERT((uiSize & ((1ULL<<uiLog2Quantum)-1)) == 0);

	_CheckAnnotationLength(pszAnnotation);

	/* Pass only the PMR flags down */
	uiPMRFlags = uiFlags & PVRSRV_MEMALLOCFLAGS_PMRFLAGSMASK;
	eError = BridgePhysmemNewRamBackedPMR(GetBridgeHandle(hDevConnection),
			uiSize,
			uiChunkSize,
			ui32NumPhysChunks,
			ui32NumVirtChunks,
			pui32MappingTable,
			uiLog2Quantum,
			uiPMRFlags,
			OSStringNLength(pszAnnotation, DEVMEM_ANNOTATION_MAX_LEN - 1) + 1,
			pszAnnotation,
			OSGetCurrentProcessID(),
			&hPMR);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate memory for %s (%s)",
				__func__,
				pszAnnotation,
				PVRSRVGETERRORSTRING(eError)));
		goto failPMR;
	}

	_DevmemImportStructInit(psImport,
			uiSize,
			uiAlign,
			uiFlags,
			hPMR,
			bExportable ? DEVMEM_PROPERTIES_EXPORTABLE : 0);

	*ppsImport = psImport;
	return PVRSRV_OK;

	failPMR:
	_DevmemImportDiscard(psImport);
	failAlloc:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}


/*****************************************************************************
 *                    Sub allocation internals                               *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DeviceMemChangeSparse(DEVMEM_MEMDESC *psMemDesc,
		IMG_UINT32 ui32AllocPageCount,
		IMG_UINT32 *paui32AllocPageIndices,
		IMG_UINT32 ui32FreePageCount,
		IMG_UINT32 *pauiFreePageIndices,
		SPARSE_MEM_RESIZE_FLAGS uiSparseFlags)
{
	PVRSRV_ERROR eError = PVRSRV_ERROR_INVALID_PARAMS;
	DEVMEM_IMPORT *psImport = psMemDesc->psImport;
	SHARED_DEV_CONNECTION hDevConnection;
	IMG_HANDLE hPMR;
	IMG_HANDLE hSrvDevMemHeap;
	POS_LOCK hLock;
	IMG_DEV_VIRTADDR sDevVAddr;
	IMG_CPU_VIRTADDR pvCpuVAddr;

	if (NULL == psImport)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid Sparse memory import", __func__));
		goto e0;
	}

	hDevConnection = psImport->hDevConnection;
	hPMR = psImport->hPMR;
	hLock = psImport->hLock;
	sDevVAddr = psImport->sDeviceImport.sDevVAddr;
	pvCpuVAddr = psImport->sCPUImport.pvCPUVAddr;

	if (NULL == hDevConnection)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid Bridge handle", __func__));
		goto e0;
	}

	if (NULL == hPMR)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid PMR handle", __func__));
		goto e0;
	}

	if ((uiSparseFlags & SPARSE_RESIZE_BOTH) && (0 == sDevVAddr.uiAddr))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid Device Virtual Map", __func__));
		goto e0;
	}

	if ((uiSparseFlags & SPARSE_MAP_CPU_ADDR) && (NULL == pvCpuVAddr))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid CPU Virtual Map", __func__));
		goto e0;
	}

	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_SECURE)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Secure buffers currently do not support sparse changes",
				__func__));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_NO_LAYOUT_CHANGE)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: This memory descriptor doesn't support sparse changes",
				__func__));
		eError = PVRSRV_ERROR_INVALID_REQUEST;
		goto e0;
	}

	hSrvDevMemHeap = psImport->sDeviceImport.psHeap->hDevMemServerHeap;

	OSLockAcquire(hLock);

	eError = BridgeChangeSparseMem(GetBridgeHandle(hDevConnection),
			hSrvDevMemHeap,
			hPMR,
			ui32AllocPageCount,
			paui32AllocPageIndices,
			ui32FreePageCount,
			pauiFreePageIndices,
			uiSparseFlags,
			psImport->uiFlags,
			sDevVAddr,
			(IMG_UINT64)((uintptr_t)pvCpuVAddr));

	OSLockRelease(hLock);

	if (GetInfoPageDebugFlags(psMemDesc->psImport->hDevConnection) & DEBUG_FEATURE_PAGE_FAULT_DEBUG_ENABLED)
	{
		BridgeDevicememHistorySparseChange(GetBridgeHandle(psMemDesc->psImport->hDevConnection),
				psMemDesc->psImport->hPMR,
				psMemDesc->uiOffset,
				psMemDesc->sDeviceMemDesc.sDevVAddr,
				psMemDesc->uiAllocSize,
				psMemDesc->szText,
				DevmemGetHeapLog2PageSize(psImport->sDeviceImport.psHeap),
				ui32AllocPageCount,
				paui32AllocPageIndices,
				ui32FreePageCount,
				pauiFreePageIndices,
				psMemDesc->ui32AllocationIndex,
				&psMemDesc->ui32AllocationIndex);
	}

#ifdef PVRSRV_UNMAP_ON_SPARSE_CHANGE
	if ((PVRSRV_OK == eError) && (psMemDesc->sCPUMemDesc.ui32RefCount))
	{
		/*
		 * Release the CPU Virtual mapping here
		 * the caller is supposed to map entire range again
		 */
		DevmemReleaseCpuVirtAddr(psMemDesc);
	}
#endif

	e0:
	return eError;
}

static void
_FreeDeviceMemory(DEVMEM_IMPORT *psImport)
{
	_DevmemImportStructRelease(psImport);
}

static PVRSRV_ERROR
_SubAllocImportAlloc(RA_PERARENA_HANDLE hArena,
		RA_LENGTH_T uiSize,
		RA_FLAGS_T _flags,
		const IMG_CHAR *pszAnnotation,
		/* returned data */
		RA_BASE_T *puiBase,
		RA_LENGTH_T *puiActualSize,
		RA_PERISPAN_HANDLE *phImport)
{
	/* When suballocations need a new lump of memory, the RA calls
	   back here.  Later, in the kernel, we must construct a new PMR
	   and a pairing between the new lump of virtual memory and the
	   PMR (whether or not such PMR is backed by physical memory) */
	DEVMEM_HEAP *psHeap;
	DEVMEM_IMPORT *psImport;
	IMG_DEVMEM_ALIGN_T uiAlign;
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32MappingTable = 0;
	DEVMEM_FLAGS_T uiFlags = (DEVMEM_FLAGS_T) _flags;
	IMG_UINT64 ui64OptionalMapAddress = DEVICEMEM_UTILS_NO_ADDRESS;

	/* Per-arena private handle is, for us, the heap */
	psHeap = hArena;

	/* align to the l.s.b. of the size...  e.g. 96kiB aligned to
	   32kiB. NB: There is an argument to say that the RA should never
	   ask us for Non-power-of-2 size anyway, but I don't want to make
	   that restriction arbitrarily now */
	uiAlign = uiSize & ~(uiSize-1);

	/* Technically this is only required for guest drivers due to
	   fw heaps being pre-allocated and pre-mapped resulting in
	   a 1:1 (i.e. virtual : physical) offset correlation but we
	   force this behaviour for all drivers to maintain consistency
	   (i.e. heap->VA uiAlign <= heap->PA uiLog2Quantum) */
	if (uiAlign > (IMG_DEVMEM_ALIGN_T)(1ULL << psHeap->uiLog2Quantum))
	{
		uiAlign = (IMG_DEVMEM_ALIGN_T)(1ULL << psHeap->uiLog2Quantum);
	}

	/* The RA should not have invoked us with a size that is not a
	   multiple of the quantum anyway */
	PVR_ASSERT((uiSize & ((1ULL<<psHeap->uiLog2Quantum)-1)) == 0);

	eError = _AllocateDeviceMemory(psHeap->psCtx->hDevConnection,
			psHeap->uiLog2Quantum,
			uiSize,
			uiSize,
			1,
			1,
			&ui32MappingTable,
			uiAlign,
			uiFlags,
			IMG_FALSE,
			"PMR sub-allocated",
			&psImport);
	if (eError != PVRSRV_OK)
	{
		goto failAlloc;
	}

#if defined(PDUMP) && defined(DEBUG)
#if defined(__KERNEL__)
	PDUMPCOMMENTWITHFLAGS(PDUMP_CONT,
			"Created PMR for sub-allocations with handle ID: 0x%p Annotation: \"%s\" (PID %u)",
			psImport->hPMR, pszAnnotation, OSGetCurrentProcessID());
#else
	PDUMPCOMMENTF(psHeap->psCtx->hDevConnection, PDUMP_FLAGS_CONTINUOUS,
			"Created PMR for sub-allocations with handle ID: %p Annotation: \"%s\" (PID %u)",
			psImport->hPMR, pszAnnotation, OSGetCurrentProcessID());
#endif
#else
	PVR_UNREFERENCED_PARAMETER(pszAnnotation);
#endif

#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
#if defined(__KERNEL__)
		PVRSRV_DEVICE_NODE *psDevNode = (PVRSRV_DEVICE_NODE *)psHeap->psCtx->hDevConnection;
		PVRSRV_RGXDEV_INFO *psDevInfo = (PVRSRV_RGXDEV_INFO *) psDevNode->pvDevice;

		PVR_ASSERT(PVRSRV_CHECK_FW_LOCAL(uiFlags));

		/* If allocation is made by the Kernel from the firmware heap, account for it
		 * under the PVR_SYS_ALLOC_PID.
		 */
		if ((psHeap == psDevInfo->psFirmwareMainHeap) || (psHeap == psDevInfo->psFirmwareConfigHeap))
		{
			eError = BridgeRIWritePMREntryWithOwner (GetBridgeHandle(psImport->hDevConnection),
					psImport->hPMR,
					PVR_SYS_ALLOC_PID);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWritePMREntryWithOwner failed (Error=%d)", __func__, eError));
			}
		}
		else
#endif
		{
			eError = BridgeRIWritePMREntry (GetBridgeHandle(psImport->hDevConnection),
					psImport->hPMR);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWritePMREntry failed (Error=%d)", __func__, eError));
			}
		}
	}
#endif

	/*
		Suballocations always get mapped into the device was we need to
		key the RA off something and as we can't export suballocations
		there is no valid reason to request an allocation an not map it
	 */
	eError = _DevmemImportStructDevMap(psHeap,
			IMG_TRUE,
			psImport,
			ui64OptionalMapAddress);
	if (eError != PVRSRV_OK)
	{
		goto failMap;
	}

	/* Mark this import struct as zeroed so we can save some PDump LDBs
	 * and do not have to CPU map + memset()*/
	if (uiFlags & PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC)
	{
		psImport->uiProperties |= DEVMEM_PROPERTIES_IMPORT_IS_ZEROED;
	}
	else if (uiFlags & PVRSRV_MEMALLOCFLAG_POISON_ON_ALLOC)
	{
		psImport->uiProperties |= DEVMEM_PROPERTIES_IMPORT_IS_POISONED;
	}
	psImport->uiProperties |= DEVMEM_PROPERTIES_IMPORT_IS_CLEAN;

	*puiBase = psImport->sDeviceImport.sDevVAddr.uiAddr;
	*puiActualSize = uiSize;
	*phImport = psImport;

	return PVRSRV_OK;

	/* error exit paths follow */

	failMap:
	_FreeDeviceMemory(psImport);
	failAlloc:

	return eError;
}

static void
_SubAllocImportFree(RA_PERARENA_HANDLE hArena,
		RA_BASE_T uiBase,
		RA_PERISPAN_HANDLE hImport)
{
	DEVMEM_IMPORT *psImport = hImport;

	PVR_ASSERT(psImport != NULL);
	PVR_ASSERT(hArena == psImport->sDeviceImport.psHeap);
	PVR_ASSERT(uiBase == psImport->sDeviceImport.sDevVAddr.uiAddr);

	_DevmemImportStructDevUnmap(psImport);
	_DevmemImportStructRelease(psImport);
}

/*****************************************************************************
 *                    Devmem context internals                               *
 *****************************************************************************/

static PVRSRV_ERROR
_PopulateContextFromBlueprint(struct _DEVMEM_CONTEXT_ *psCtx,
		DEVMEM_HEAPCFGID uiHeapBlueprintID)
{
	PVRSRV_ERROR eError;
	PVRSRV_ERROR eError2;
	struct _DEVMEM_HEAP_ **ppsHeapArray;
	IMG_UINT32 uiNumHeaps;
	IMG_UINT32 uiHeapsToUnwindOnError;
	IMG_UINT32 uiHeapIndex;
	IMG_DEV_VIRTADDR sDevVAddrBase;
	IMG_CHAR aszHeapName[DEVMEM_HEAPNAME_MAXLENGTH];
	IMG_DEVMEM_SIZE_T uiHeapLength;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2DataPageSize;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2ImportAlignment;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2TilingStrideFactor;

	eError = DevmemHeapCount(psCtx->hDevConnection,
			uiHeapBlueprintID,
			&uiNumHeaps);
	if (eError != PVRSRV_OK)
	{
		goto e0;
	}

	if (uiNumHeaps == 0)
	{
		ppsHeapArray = NULL;
	}
	else
	{
		ppsHeapArray = OSAllocMem(sizeof(*ppsHeapArray) * uiNumHeaps);
		if (ppsHeapArray == NULL)
		{
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto e0;
		}
	}

	uiHeapsToUnwindOnError = 0;

	for (uiHeapIndex = 0; uiHeapIndex < uiNumHeaps; uiHeapIndex++)
	{
		eError = DevmemHeapDetails(psCtx->hDevConnection,
				uiHeapBlueprintID,
				uiHeapIndex,
				&aszHeapName[0],
				sizeof(aszHeapName),
				&sDevVAddrBase,
				&uiHeapLength,
				&uiLog2DataPageSize,
				&uiLog2ImportAlignment,
				&uiLog2TilingStrideFactor);
		if (eError != PVRSRV_OK)
		{
			goto e1;
		}

		eError = DevmemCreateHeap(psCtx,
				sDevVAddrBase,
				uiHeapLength,
				uiLog2DataPageSize,
				uiLog2ImportAlignment,
				uiLog2TilingStrideFactor,
				aszHeapName,
				uiHeapBlueprintID,
				&ppsHeapArray[uiHeapIndex]);
		if (eError != PVRSRV_OK)
		{
			goto e1;
		}

		uiHeapsToUnwindOnError = uiHeapIndex + 1;
	}

	psCtx->uiAutoHeapCount = uiNumHeaps;
	psCtx->ppsAutoHeapArray = ppsHeapArray;

	PVR_ASSERT(psCtx->uiNumHeaps >= psCtx->uiAutoHeapCount);
	PVR_ASSERT(psCtx->uiAutoHeapCount == uiNumHeaps);

	return PVRSRV_OK;

	/* error exit paths */
	e1:
	for (uiHeapIndex = 0; uiHeapIndex < uiHeapsToUnwindOnError; uiHeapIndex++)
	{
		eError2 = DevmemDestroyHeap(ppsHeapArray[uiHeapIndex]);
		PVR_ASSERT(eError2 == PVRSRV_OK);
	}

	if (uiNumHeaps != 0)
	{
		OSFreeMem(ppsHeapArray);
	}

	e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static PVRSRV_ERROR
_UnpopulateContextFromBlueprint(struct _DEVMEM_CONTEXT_ *psCtx)
{
	PVRSRV_ERROR eReturn = PVRSRV_OK;
	PVRSRV_ERROR eError2;
	IMG_UINT32 uiHeapIndex;
	IMG_BOOL bDoCheck = IMG_TRUE;
#if defined(__KERNEL__)
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	if (psPVRSRVData->eServicesState != PVRSRV_SERVICES_STATE_OK)
	{
		bDoCheck = IMG_FALSE;
	}
#endif

	for (uiHeapIndex = 0; uiHeapIndex < psCtx->uiAutoHeapCount; uiHeapIndex++)
	{
		if (!psCtx->ppsAutoHeapArray[uiHeapIndex])
		{
			continue;
		}

		eError2 = DevmemDestroyHeap(psCtx->ppsAutoHeapArray[uiHeapIndex]);
		if (eError2 != PVRSRV_OK)
		{
			eReturn = eError2;
		}
		else
		{
			psCtx->ppsAutoHeapArray[uiHeapIndex] = NULL;
		}
	}

	if ((!bDoCheck || (eReturn == PVRSRV_OK)) && psCtx->ppsAutoHeapArray)
	{
		OSFreeMem(psCtx->ppsAutoHeapArray);
		psCtx->ppsAutoHeapArray = NULL;
		psCtx->uiAutoHeapCount = 0;
	}

	return eReturn;
}

static PVRSRV_ERROR
_AllocateMCUFenceAddress(struct _DEVMEM_CONTEXT_ *psCtx)
{
	PVRSRV_ERROR		eError;
	DEVMEM_HEAP			*psGeneralHeap;
	IMG_DEV_VIRTADDR	sTempMCUFenceAddr;

	eError = DevmemFindHeapByName(psCtx, RGX_GENERAL_HEAP_IDENT, &psGeneralHeap);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: General Heap not found (%s)", __func__, GET_ERROR_STRING(eError)));
		goto e0;
	}

	/* MCUFence: Fixed address reserved per Memory Context */
	eError = DevmemAllocate(psGeneralHeap,
			sizeof(IMG_UINT32),
			RGX_CR_MCU_FENCE_ADDR_ALIGNSIZE,
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE,
			"MCUFence",
			&psCtx->psMCUFenceMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate MCU fence word (%s)", __func__, GET_ERROR_STRING(eError)));
		goto e0;
	}

	/* This is the first memory allocation on General Heap so its virtual address
	 * is always equal to heap base address. Storing this address separately is not required. */
	eError = DevmemMapToDevice(psCtx->psMCUFenceMemDesc, psGeneralHeap, &sTempMCUFenceAddr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to map MCU fence word (%s)", __func__, GET_ERROR_STRING(eError)));
		goto e1;
	}
	else if (sTempMCUFenceAddr.uiAddr != psGeneralHeap->sBaseAddress.uiAddr)
	{

		PVR_DPF((PVR_DBG_ERROR,
				 "%s: MCU_FENCE address (%" IMG_UINT64_FMTSPECx ") "
				 "not at the start of General Heap (%" IMG_UINT64_FMTSPECx ")",
				__func__, sTempMCUFenceAddr.uiAddr,
				 psGeneralHeap->sBaseAddress.uiAddr));
		eError = PVRSRV_ERROR_DEVICEMEM_MAP_FAILED;
		goto e1;
	}

	e0:
	return eError;

	e1:
	DevmemFree(psCtx->psMCUFenceMemDesc);
	psCtx->psMCUFenceMemDesc = NULL;
	return eError;
}

/*****************************************************************************
 *                    Devmem context functions                               *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DevmemCreateContext(SHARED_DEV_CONNECTION hDevConnection,
		DEVMEM_HEAPCFGID uiHeapBlueprintID,
		IMG_BOOL bMCUFenceAllocation,
		DEVMEM_CONTEXT **ppsCtxPtr)
{
	PVRSRV_ERROR		eError;
	DEVMEM_CONTEXT		*psCtx;
	/* handle to the server-side counterpart of the device memory
	   context (specifically, for handling mapping to device MMU) */
	IMG_HANDLE			hDevMemServerContext;
	IMG_HANDLE			hPrivData;
	IMG_BOOL			bHeapCfgMetaId = (uiHeapBlueprintID == DEVMEM_HEAPCFG_META);

	if (ppsCtxPtr == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	psCtx = OSAllocMem(sizeof *psCtx);
	if (psCtx == NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	psCtx->uiNumHeaps = 0;

	psCtx->hDevConnection = hDevConnection;

	/* Create (server-side) Device Memory context */
	eError = BridgeDevmemIntCtxCreate(GetBridgeHandle(psCtx->hDevConnection),
			bHeapCfgMetaId,
			&hDevMemServerContext,
			&hPrivData,
			&psCtx->ui32CPUCacheLineSize);
	if (eError != PVRSRV_OK) goto e1;

	psCtx->hDevMemServerContext = hDevMemServerContext;
	psCtx->hPrivData = hPrivData;

	/* automagic heap creation */
	psCtx->uiAutoHeapCount = 0;

	eError = _PopulateContextFromBlueprint(psCtx, uiHeapBlueprintID);
	if (eError != PVRSRV_OK) goto e2;

	/* Allocate a word at the start of the General heap to be used as MCU_FENCE Address */
	if (uiHeapBlueprintID == DEVMEM_HEAPCFG_FORCLIENTS  &&  bMCUFenceAllocation)
	{
		eError = _AllocateMCUFenceAddress(psCtx);
		if (eError != PVRSRV_OK) goto e2;
	}
	else
	{
		psCtx->psMCUFenceMemDesc = NULL;
	}

	*ppsCtxPtr = psCtx;

	PVR_ASSERT(psCtx->uiNumHeaps == psCtx->uiAutoHeapCount);
	return PVRSRV_OK;

	/* error exit paths follow */

	e2:
	PVR_ASSERT(psCtx->uiAutoHeapCount == 0);
	PVR_ASSERT(psCtx->uiNumHeaps == 0);
	BridgeDevmemIntCtxDestroy(GetBridgeHandle(psCtx->hDevConnection), hDevMemServerContext);

	e1:
	OSFreeMem(psCtx);

	e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAcquireDevPrivData(DEVMEM_CONTEXT *psCtx,
		IMG_HANDLE *hPrivData)
{
	PVRSRV_ERROR eError;

	if ((psCtx == NULL) || (hPrivData == NULL))
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	*hPrivData = psCtx->hPrivData;
	return PVRSRV_OK;

	e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemReleaseDevPrivData(DEVMEM_CONTEXT *psCtx)
{
	PVRSRV_ERROR eError;

	if (psCtx == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}
	return PVRSRV_OK;

	e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}


IMG_INTERNAL PVRSRV_ERROR
DevmemFindHeapByName(const struct _DEVMEM_CONTEXT_ *psCtx,
		const IMG_CHAR *pszHeapName,
		struct _DEVMEM_HEAP_ **ppsHeapRet)
{
	IMG_UINT32 uiHeapIndex;

	/* N.B.  This func is only useful for finding "automagic" heaps by name */
	for (uiHeapIndex = 0;
			uiHeapIndex < psCtx->uiAutoHeapCount;
			uiHeapIndex++)
	{
		if (!OSStringCompare(psCtx->ppsAutoHeapArray[uiHeapIndex]->pszName, pszHeapName))
		{
			*ppsHeapRet = psCtx->ppsAutoHeapArray[uiHeapIndex];
			return PVRSRV_OK;
		}
	}

	return PVRSRV_ERROR_DEVICEMEM_INVALID_HEAP_INDEX;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemDestroyContext(DEVMEM_CONTEXT *psCtx)
{
	PVRSRV_ERROR eError;
	IMG_BOOL bDoCheck = IMG_TRUE;

#if defined(__KERNEL__)
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	if (psPVRSRVData->eServicesState != PVRSRV_SERVICES_STATE_OK)
	{
		bDoCheck = IMG_FALSE;
	}
#endif

	if (psCtx == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if (psCtx->psMCUFenceMemDesc != NULL)
	{
		DevmemReleaseDevVirtAddr(psCtx->psMCUFenceMemDesc);
		DevmemFree(psCtx->psMCUFenceMemDesc);
	}

	eError = _UnpopulateContextFromBlueprint(psCtx);
	if (bDoCheck && eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: _UnpopulateContextFromBlueprint failed (%d) leaving %d heaps",
				__func__, eError, psCtx->uiNumHeaps));
		goto e1;
	}

	eError = DestroyServerResource(psCtx->hDevConnection,
	                               NULL,
	                               BridgeDevmemIntCtxDestroy,
	                               psCtx->hDevMemServerContext);
	if (bDoCheck && eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: BridgeDevmemIntCtxDestroy failed (%d)",
				__func__, eError));
		goto e1;
	}

	/* should be no more heaps left */
	if (bDoCheck && psCtx->uiNumHeaps)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Additional heaps remain in DEVMEM_CONTEXT",
				__func__));
		eError = PVRSRV_ERROR_DEVICEMEM_ADDITIONAL_HEAPS_IN_CONTEXT;
		goto e1;
	}

	OSDeviceMemSet(psCtx, 0, sizeof(*psCtx));
	OSFreeMem(psCtx);

	e1:
	return eError;
}

/*****************************************************************************
 *                 Devmem heap query functions                               *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DevmemHeapConfigCount(SHARED_DEV_CONNECTION hDevConnection,
		IMG_UINT32 *puiNumHeapConfigsOut)
{
	PVRSRV_ERROR eError;
	eError = BridgeHeapCfgHeapConfigCount(GetBridgeHandle(hDevConnection),
			puiNumHeapConfigsOut);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemHeapCount(SHARED_DEV_CONNECTION hDevConnection,
		IMG_UINT32 uiHeapConfigIndex,
		IMG_UINT32 *puiNumHeapsOut)
{
	PVRSRV_ERROR eError;
	eError = BridgeHeapCfgHeapCount(GetBridgeHandle(hDevConnection),
			uiHeapConfigIndex,
			puiNumHeapsOut);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemHeapConfigName(SHARED_DEV_CONNECTION hDevConnection,
		IMG_UINT32 uiHeapConfigIndex,
		IMG_CHAR *pszConfigNameOut,
		IMG_UINT32 uiConfigNameBufSz)
{
	PVRSRV_ERROR eError;
	eError = BridgeHeapCfgHeapConfigName(GetBridgeHandle(hDevConnection),
			uiHeapConfigIndex,
			uiConfigNameBufSz,
			pszConfigNameOut);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemHeapDetails(SHARED_DEV_CONNECTION hDevConnection,
		IMG_UINT32 uiHeapConfigIndex,
		IMG_UINT32 uiHeapIndex,
		IMG_CHAR *pszHeapNameOut,
		IMG_UINT32 uiHeapNameBufSz,
		IMG_DEV_VIRTADDR *psDevVAddrBaseOut,
		IMG_DEVMEM_SIZE_T *puiHeapLengthOut,
		IMG_UINT32 *puiLog2DataPageSizeOut,
		IMG_UINT32 *puiLog2ImportAlignmentOut,
		IMG_UINT32 *puiLog2TilingStrideFactor)
{
	PVRSRV_ERROR eError;

	eError = BridgeHeapCfgHeapDetails(GetBridgeHandle(hDevConnection),
			uiHeapConfigIndex,
			uiHeapIndex,
			uiHeapNameBufSz,
			pszHeapNameOut,
			psDevVAddrBaseOut,
			puiHeapLengthOut,
			puiLog2DataPageSizeOut,
			puiLog2ImportAlignmentOut,
			puiLog2TilingStrideFactor);

	VG_MARK_INITIALIZED(pszHeapNameOut,uiHeapNameBufSz);

	return eError;
}

/*****************************************************************************
 *                    Devmem heap functions                                  *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DevmemGetHeapInt(DEVMEM_HEAP *psHeap,
		IMG_HANDLE *phDevmemHeap)
{
	if (psHeap == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	*phDevmemHeap = psHeap->hDevMemServerHeap;
	return PVRSRV_OK;
}

/* See devicemem.h for important notes regarding the arguments
   to this function */
IMG_INTERNAL PVRSRV_ERROR
DevmemCreateHeap(DEVMEM_CONTEXT *psCtx,
		IMG_DEV_VIRTADDR sBaseAddress,
		IMG_DEVMEM_SIZE_T uiLength,
		IMG_UINT32 ui32Log2Quantum,
		IMG_UINT32 ui32Log2ImportAlignment,
		IMG_UINT32 ui32Log2TilingStrideFactor,
		const IMG_CHAR *pszName,
		DEVMEM_HEAPCFGID uiHeapBlueprintID,
		DEVMEM_HEAP **ppsHeapPtr)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_ERROR eError2;
	DEVMEM_HEAP *psHeap;
	/* handle to the server-side counterpart of the device memory heap
	  (specifically, for handling mapping to device MMU */
	IMG_HANDLE hDevMemServerHeap;
	IMG_BOOL bRANoSplit = IMG_FALSE;

	IMG_CHAR aszBuf[100];
	IMG_CHAR *pszStr;

	if (ppsHeapPtr == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	psHeap = OSAllocMem(sizeof *psHeap);
	if (psHeap == NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	/* Need to keep local copy of heap name, so caller may free theirs */
	pszStr = OSAllocMem(OSStringLength(pszName)+1);
	if (pszStr == NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e1;
	}
	OSStringCopy(pszStr, pszName);
	psHeap->pszName = pszStr;

	psHeap->uiSize = uiLength;
	psHeap->sBaseAddress = sBaseAddress;
	OSAtomicWrite(&psHeap->hImportCount,0);

	OSSNPrintf(aszBuf, sizeof(aszBuf),
			"NDM heap '%s' (suballocs) ctx:%p",
			pszName, psCtx);
	pszStr = OSAllocMem(OSStringLength(aszBuf)+1);
	if (pszStr == NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e2;
	}
	OSStringCopy(pszStr, aszBuf);
	psHeap->pszSubAllocRAName = pszStr;

#if defined(PDUMP)
	/* The META heap is shared globally so a single physical memory import
	 * may be used to satisfy allocations of different processes.
	 * This is problematic when PDumping because the physical memory
	 * import used to satisfy a new allocation may actually have been
	 * imported (and thus the PDump MALLOC generated) before the PDump
	 * client was started, leading to the MALLOC being missing.
	 *
	 * This is solved by disabling splitting of imports for the META physmem
	 * RA, meaning that every firmware allocation gets its own import, thus
	 * ensuring the MALLOC is present for every allocation made within the
	 * pdump capture range
	 */
	if (uiHeapBlueprintID == DEVMEM_HEAPCFG_META)
	{
		bRANoSplit = IMG_TRUE;
	}
#else
	PVR_UNREFERENCED_PARAMETER(uiHeapBlueprintID);
#endif


	psHeap->psSubAllocRA = RA_Create(psHeap->pszSubAllocRAName,
			/* Subsequent imports: */
			ui32Log2Quantum,
			RA_LOCKCLASS_2,
			_SubAllocImportAlloc,
			_SubAllocImportFree,
			(RA_PERARENA_HANDLE) psHeap,
			bRANoSplit);
	if (psHeap->psSubAllocRA == NULL)
	{
		eError = PVRSRV_ERROR_DEVICEMEM_UNABLE_TO_CREATE_ARENA;
		goto e3;
	}

	psHeap->uiLog2ImportAlignment = ui32Log2ImportAlignment;
	psHeap->uiLog2TilingStrideFactor = ui32Log2TilingStrideFactor;
	psHeap->uiLog2Quantum = ui32Log2Quantum;

	if (!OSStringCompare(pszName, RGX_GENERAL_SVM_HEAP_IDENT))
	{
		/* The SVM heap normally starts out as this type though
		   it may transition to DEVMEM_HEAP_TYPE_USER_MANAGED
		   on platforms with more processor virtual address
		   bits than device virtual address bits */
		psHeap->eHeapType = DEVMEM_HEAP_TYPE_KERNEL_MANAGED;
	}
	else
	{
		psHeap->eHeapType = DEVMEM_HEAP_TYPE_UNKNOWN;
	}

	OSSNPrintf(aszBuf, sizeof(aszBuf),
			"NDM heap '%s' (QVM) ctx:%p",
			pszName, psCtx);
	pszStr = OSAllocMem(OSStringLength(aszBuf)+1);
	if (pszStr == NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e4;
	}
	OSStringCopy(pszStr, aszBuf);
	psHeap->pszQuantizedVMRAName = pszStr;

	psHeap->psQuantizedVMRA = RA_Create(psHeap->pszQuantizedVMRAName,
			/* Subsequent import: */
			0, RA_LOCKCLASS_1, NULL, NULL,
			(RA_PERARENA_HANDLE) psHeap,
			IMG_FALSE);
	if (psHeap->psQuantizedVMRA == NULL)
	{
		eError = PVRSRV_ERROR_DEVICEMEM_UNABLE_TO_CREATE_ARENA;
		goto e5;
	}

	if (!RA_Add(psHeap->psQuantizedVMRA,
			(RA_BASE_T)sBaseAddress.uiAddr,
			(RA_LENGTH_T)uiLength,
			(RA_FLAGS_T)0, /* This RA doesn't use or need flags */
			NULL /* per ispan handle */))
	{
		RA_Delete(psHeap->psQuantizedVMRA);
		eError = PVRSRV_ERROR_DEVICEMEM_UNABLE_TO_CREATE_ARENA;
		goto e5;
	}

	psHeap->psCtx = psCtx;


	/* Create server-side counterpart of Device Memory heap */
	eError = BridgeDevmemIntHeapCreate(GetBridgeHandle(psCtx->hDevConnection),
			psCtx->hDevMemServerContext,
			sBaseAddress,
			uiLength,
			ui32Log2Quantum,
			&hDevMemServerHeap);
	if (eError != PVRSRV_OK)
	{
		goto e6;
	}
	psHeap->hDevMemServerHeap = hDevMemServerHeap;

	eError = OSLockCreate(&psHeap->hLock);
	if (eError != PVRSRV_OK)
	{
		goto e7;
	}

	psHeap->psCtx->uiNumHeaps ++;
	*ppsHeapPtr = psHeap;

#if defined PVRSRV_NEWDEVMEM_SUPPORT_MEM_TRACKING
	psHeap->psMemDescList = NULL;
#endif /* PVRSRV_NEWDEVMEM_SUPPORT_MEM_TRACKING */

	return PVRSRV_OK;

	/* error exit paths */
	e7:
	eError2 = BridgeDevmemIntHeapDestroy(GetBridgeHandle(psCtx->hDevConnection),
			psHeap->hDevMemServerHeap);
	PVR_ASSERT (eError2 == PVRSRV_OK);
	e6:
	if (psHeap->psQuantizedVMRA)
		RA_Delete(psHeap->psQuantizedVMRA);
	e5:
	if (psHeap->pszQuantizedVMRAName)
		OSFreeMem(psHeap->pszQuantizedVMRAName);
	e4:
	RA_Delete(psHeap->psSubAllocRA);
	e3:
	OSFreeMem(psHeap->pszSubAllocRAName);
	e2:
	OSFreeMem(psHeap->pszName);
	e1:
	OSFreeMem(psHeap);
	e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemGetHeapBaseDevVAddr(struct _DEVMEM_HEAP_ *psHeap,
		IMG_DEV_VIRTADDR *pDevVAddr)
{
	if (psHeap == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	*pDevVAddr = psHeap->sBaseAddress;

	return PVRSRV_OK;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemExportalignAdjustSizeAndAlign(IMG_UINT32 uiLog2Quantum,
		IMG_DEVMEM_SIZE_T *puiSize,
		IMG_DEVMEM_ALIGN_T *puiAlign)
{
	IMG_DEVMEM_SIZE_T uiSize = *puiSize;
	IMG_DEVMEM_ALIGN_T uiAlign = *puiAlign;

	/* Just in case someone changes definition of IMG_DEVMEM_ALIGN_T. */
	static_assert(sizeof(unsigned long long) == sizeof(uiAlign),
	              "invalid uiAlign size");
	/* This value is used for shifting so it cannot be greater than number
	 * of bits in unsigned long long (sizeof(1ULL)). Using greater value is
	 * undefined behaviour. */
	if (uiLog2Quantum >= sizeof(unsigned long long) * 8)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if ((1ULL << uiLog2Quantum) > uiAlign)
	{
		uiAlign = 1ULL << uiLog2Quantum;
	}
	uiSize = (uiSize + uiAlign - 1) & ~(uiAlign - 1);

	*puiSize = uiSize;
	*puiAlign = uiAlign;

	return PVRSRV_OK;
}


IMG_INTERNAL PVRSRV_ERROR
DevmemDestroyHeap(DEVMEM_HEAP *psHeap)
{
	PVRSRV_ERROR eError;
	IMG_INT uiImportCount;
#if defined(PVRSRV_FORCE_UNLOAD_IF_BAD_STATE)
	IMG_BOOL bDoCheck = IMG_TRUE;
#if defined(__KERNEL__)
	if (PVRSRVGetPVRSRVData()->eServicesState != PVRSRV_SERVICES_STATE_OK)
	{
		bDoCheck = IMG_FALSE;
	}
#endif
#endif

	if (psHeap == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	uiImportCount = OSAtomicRead(&psHeap->hImportCount);
	if (uiImportCount > 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "%d(%s) leaks remain", uiImportCount, psHeap->pszName));
#if defined(__KERNEL__)
#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
		PVR_DPF((PVR_DBG_ERROR, "Details of remaining allocated device memory (for all processes):"));
		RIDumpAllKM();
#else
		PVR_DPF((PVR_DBG_ERROR, "Compile with PVRSRV_ENABLE_GPU_MEMORY_INFO=1 to get a full "
				"list of all driver allocations."));
#endif
#endif
#if defined(PVRSRV_FORCE_UNLOAD_IF_BAD_STATE)
		if (bDoCheck)
#endif
		{
			return PVRSRV_ERROR_DEVICEMEM_ALLOCATIONS_REMAIN_IN_HEAP;
		}
	}

	eError = DestroyServerResource(psHeap->psCtx->hDevConnection,
	                               NULL,
	                               BridgeDevmemIntHeapDestroy,
	                               psHeap->hDevMemServerHeap);

#if defined(PVRSRV_FORCE_UNLOAD_IF_BAD_STATE)
	if (bDoCheck)
#endif
	{
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					"%s: BridgeDevmemIntHeapDestroy failed (%d)",
					__func__, eError));
			return eError;
		}
	}

	PVR_ASSERT(psHeap->psCtx->uiNumHeaps > 0);
	psHeap->psCtx->uiNumHeaps--;

	OSLockDestroy(psHeap->hLock);

	if (psHeap->psQuantizedVMRA)
	{
		RA_Delete(psHeap->psQuantizedVMRA);
	}
	if (psHeap->pszQuantizedVMRAName)
	{
		OSFreeMem(psHeap->pszQuantizedVMRAName);
	}

	RA_Delete(psHeap->psSubAllocRA);
	OSFreeMem(psHeap->pszSubAllocRAName);

	OSFreeMem(psHeap->pszName);

	OSDeviceMemSet(psHeap, 0, sizeof(*psHeap));
	OSFreeMem(psHeap);

	return PVRSRV_OK;
}

/*****************************************************************************
 *                Devmem allocation/free functions                           *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DevmemSubAllocate(IMG_UINT8 uiPreAllocMultiplier,
		DEVMEM_HEAP *psHeap,
		IMG_DEVMEM_SIZE_T uiSize,
		IMG_DEVMEM_ALIGN_T uiAlign,
		DEVMEM_FLAGS_T uiFlags,
		const IMG_CHAR *pszText,
		DEVMEM_MEMDESC **ppsMemDescPtr)
{
	RA_BASE_T uiAllocatedAddr = 0;
	RA_LENGTH_T uiAllocatedSize;
	RA_PERISPAN_HANDLE hImport; /* the "import" from which this sub-allocation came */
	PVRSRV_ERROR eError;
	DEVMEM_MEMDESC *psMemDesc = NULL;
	IMG_DEVMEM_OFFSET_T uiOffset = 0;
	DEVMEM_IMPORT *psImport;
	IMG_UINT32 ui32CPUCacheLineSize;
	void *pvAddr = NULL;

	IMG_BOOL bImportClean;
	IMG_BOOL bCPUCleanFlag = PVRSRV_CHECK_CPU_CACHE_CLEAN(uiFlags);
	IMG_BOOL bZero = PVRSRV_CHECK_ZERO_ON_ALLOC(uiFlags);
	IMG_BOOL bPoisonOnAlloc = PVRSRV_CHECK_POISON_ON_ALLOC(uiFlags);
	IMG_BOOL bCPUCached = (PVRSRV_CHECK_CPU_CACHE_COHERENT(uiFlags) ||
			PVRSRV_CHECK_CPU_CACHE_INCOHERENT(uiFlags));
	IMG_BOOL bGPUCached = (PVRSRV_CHECK_GPU_CACHE_COHERENT(uiFlags) ||
			PVRSRV_CHECK_GPU_CACHE_INCOHERENT(uiFlags));
	PVRSRV_CACHE_OP eOp = PVRSRV_CACHE_OP_INVALIDATE;
	IMG_UINT32	ui32CacheLineSize = 0;

	if (uiFlags & PVRSRV_MEMALLOCFLAG_NO_OSPAGES_ON_ALLOC)
	{
		/* Deferred Allocation not supported on SubAllocs*/
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	if (psHeap == NULL || psHeap->psCtx == NULL ||ppsMemDescPtr == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

#if defined(__KERNEL__)
	{
		/* The hDevConnection holds two different types of pointers depending on the
		 * address space in which it is used.
		 * In this instance the variable points to the device node in server */
		PVRSRV_DEVICE_NODE *psDevNode = (PVRSRV_DEVICE_NODE *)psHeap->psCtx->hDevConnection;
		ui32CacheLineSize = GET_ROGUE_CACHE_LINE_SIZE(PVRSRV_GET_DEVICE_FEATURE_VALUE(psDevNode, SLC_CACHE_LINE_SIZE_BITS));
	}
#else
	ui32CacheLineSize = ROGUE_CACHE_LINE_SIZE;
#endif

	/* The following logic makes sure that any cached memory is aligned to both the CPU and GPU.
	 * To be aligned on both you have to take the Lowest Common Multiple (LCM) of the cache line sizes of each.
	 * As the possibilities are all powers of 2 then simply the largest number can be picked as the LCM.
	 * Therefore this algorithm just picks the highest from the CPU, GPU and given alignments.
	 */
	ui32CPUCacheLineSize = psHeap->psCtx->ui32CPUCacheLineSize;
	/* If the CPU cache line size is larger than the alignment given then it is the lowest common multiple
	 * Also checking if the allocation is going to be cached on the CPU
	 * Currently there is no check for the validity of the cache coherent option.
	 * In this case, the alignment could be applied but the mode could still fall back to uncached.
	 */
	if (ui32CPUCacheLineSize > uiAlign && bCPUCached)
	{
		uiAlign = ui32CPUCacheLineSize;
	}

	/* If the GPU cache line size is larger than the alignment given then it is the lowest common multiple
	 * Also checking if the allocation is going to be cached on the GPU via checking for any of the cached options.
	 * Currently there is no check for the validity of the cache coherent option.
	 * In this case, the alignment could be applied but the mode could still fall back to uncached.
	 */
	if (ui32CacheLineSize > uiAlign && bGPUCached)
	{
		uiAlign = ui32CacheLineSize;
	}

	eError = _DevmemValidateParams(uiSize,
			uiAlign,
			&uiFlags);
	if (eError != PVRSRV_OK)
	{
		goto failParams;
	}

	eError =_DevmemMemDescAlloc(&psMemDesc);
	if (eError != PVRSRV_OK)
	{
		goto failMemDescAlloc;
	}

	/* No request for exportable memory so use the RA */
	eError = RA_Alloc(psHeap->psSubAllocRA,
			uiSize,
			uiPreAllocMultiplier,
			uiFlags,
			uiAlign,
			pszText,
			&uiAllocatedAddr,
			&uiAllocatedSize,
			&hImport);
	if (PVRSRV_OK != eError)
	{
		goto failDeviceMemAlloc;
	}

	psImport = hImport;

	/* This assignment is assuming the RA returns an hImport where suballocations
	 * can be made from if uiSize is NOT a page multiple of the passed heap.
	 *
	 * So we check if uiSize is a page multiple and mark it as exportable
	 * if it is not.
	 * */
	if (!(uiSize & ((1ULL << psHeap->uiLog2Quantum) - 1)) &&
	    (uiPreAllocMultiplier == RA_NO_IMPORT_MULTIPLIER))
	{
		psImport->uiProperties |= DEVMEM_PROPERTIES_EXPORTABLE;
	}
	psImport->uiProperties |= DEVMEM_PROPERTIES_SUBALLOCATABLE;

	uiOffset = uiAllocatedAddr - psImport->sDeviceImport.sDevVAddr.uiAddr;

#if defined(PDUMP) && defined(DEBUG)
#if defined(__KERNEL__)
	PDUMPCOMMENTWITHFLAGS(PDUMP_CONT,
			"Suballocated %u Byte for \"%s\" from PMR with handle ID: 0x%p (PID %u)",
			(IMG_UINT32) uiSize, pszText, psImport->hPMR, OSGetCurrentProcessID());
#else
	PDUMPCOMMENTF(psHeap->psCtx->hDevConnection, PDUMP_FLAGS_CONTINUOUS,
			"Suballocated %u Byte for \"%s\" from PMR with handle ID: %p (PID %u)",
			(IMG_UINT32) uiSize,
			pszText,
			psImport->hPMR,
			OSGetCurrentProcessID());
#endif
#endif

	_DevmemMemDescInit(psMemDesc,
			uiOffset,
			psImport,
			uiSize);

	bImportClean = ((psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_IMPORT_IS_CLEAN) != 0);

	/* Zero the memory */
	if (bZero)
	{
		/* Has the import been zeroed on allocation and were no suballocations returned to it so far? */
		bImportClean = bImportClean && ((psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_IMPORT_IS_ZEROED) != 0);

		if (!bImportClean)
		{
			eOp = PVRSRV_CACHE_OP_FLUSH;

			eError = DevmemAcquireCpuVirtAddr(psMemDesc, &pvAddr);
			if (eError != PVRSRV_OK)
			{
				goto failMaintenance;
			}

			/* uiSize is a 64-bit quantity whereas the 3rd argument
			 * to OSDeviceMemSet is a 32-bit quantity on 32-bit systems
			 * hence a compiler warning of implicit cast and loss of data.
			 * Added explicit cast and assert to remove warning.
			 */
			PVR_ASSERT(uiSize < IMG_UINT32_MAX);

			OSDeviceMemSet(pvAddr, 0x0, (size_t) uiSize);
#if defined(PDUMP)
			DevmemPDumpLoadZeroMem(psMemDesc, 0, uiSize, PDUMP_FLAGS_CONTINUOUS);
#endif
		}
	}
	else if (bPoisonOnAlloc)
	{
		/* Has the import been poisoned on allocation and were no suballocations returned to it so far? */
		bPoisonOnAlloc = (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_IMPORT_IS_POISONED) != 0;

		if (!bPoisonOnAlloc)
		{
			eOp = PVRSRV_CACHE_OP_FLUSH;

			eError = DevmemAcquireCpuVirtAddr(psMemDesc, &pvAddr);
			if (eError != PVRSRV_OK)
			{
				goto failMaintenance;
			}

			if (PVRSRV_CHECK_CPU_UNCACHED(uiFlags) ||
					PVRSRV_CHECK_CPU_WRITE_COMBINE(uiFlags))
			{
				OSDeviceMemSet(pvAddr, PVRSRV_POISON_ON_ALLOC_VALUE,
						uiSize);
			}
			else
			{
				OSCachedMemSet(pvAddr, PVRSRV_POISON_ON_ALLOC_VALUE,
						uiSize);
			}

			bPoisonOnAlloc = IMG_TRUE;
		}
	}

	/* Flush or invalidate */
	if (bCPUCached && !bImportClean && (bZero || bCPUCleanFlag || bPoisonOnAlloc))
	{
		/* BridgeCacheOpQueue _may_ be deferred so use BridgeCacheOpExec
		   to ensure this cache maintenance is actioned immediately */
		eError = BridgeCacheOpExec (GetBridgeHandle(psMemDesc->psImport->hDevConnection),
				psMemDesc->psImport->hPMR,
				(IMG_UINT64)(uintptr_t)
				pvAddr - psMemDesc->uiOffset,
				psMemDesc->uiOffset,
				psMemDesc->uiAllocSize,
				eOp);
		if (eError != PVRSRV_OK)
		{
			goto failMaintenance;
		}
	}

	if (pvAddr)
	{
		DevmemReleaseCpuVirtAddr(psMemDesc);
		pvAddr = NULL;
	}

	/* copy the allocation descriptive name and size so it can be passed to DevicememHistory when
	 * the allocation gets mapped/unmapped
	 */
	_CheckAnnotationLength(pszText);
#if defined(__KERNEL__)
	OSStringLCopy(psMemDesc->szText, pszText, DEVMEM_ANNOTATION_MAX_LEN);
#else
	OSStringNCopy(psMemDesc->szText, pszText, DEVMEM_ANNOTATION_MAX_LEN);
	psMemDesc->szText[DEVMEM_ANNOTATION_MAX_LEN - 1] = '\0';
#endif	/* if defined(__KERNEL__) */

#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psMemDesc->psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
		/* Attach RI information */
		eError = BridgeRIWriteMEMDESCEntry (GetBridgeHandle(psMemDesc->psImport->hDevConnection),
				psMemDesc->psImport->hPMR,
				OSStringNLength(psMemDesc->szText, DEVMEM_ANNOTATION_MAX_LEN),
				psMemDesc->szText,
				psMemDesc->uiOffset,
				uiAllocatedSize,
				IMG_FALSE,
				IMG_TRUE,
				&(psMemDesc->hRIHandle));
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWriteMEMDESCEntry failed (Error=%d)", __func__, eError));
		}
	}
#else /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */
	PVR_UNREFERENCED_PARAMETER (pszText);
#endif /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */

	*ppsMemDescPtr = psMemDesc;

	return PVRSRV_OK;

	/* error exit paths follow */

	failMaintenance:
	if (pvAddr)
	{
		DevmemReleaseCpuVirtAddr(psMemDesc);
		pvAddr = NULL;
	}
	_DevmemMemDescRelease(psMemDesc);
	psMemDesc = NULL;	/* Make sure we don't do a discard after the release */
	failDeviceMemAlloc:
	if (psMemDesc)
	{
		_DevmemMemDescDiscard(psMemDesc);
	}
	failMemDescAlloc:
	failParams:
	PVR_ASSERT(eError != PVRSRV_OK);
	PVR_DPF((PVR_DBG_ERROR,
			"%s: Failed! Error is %s. Allocation size: " IMG_DEVMEM_SIZE_FMTSPEC,
			__func__,
			PVRSRVGETERRORSTRING(eError),
			uiSize));
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAllocateExportable(SHARED_DEV_CONNECTION hDevConnection,
		IMG_DEVMEM_SIZE_T uiSize,
		IMG_DEVMEM_ALIGN_T uiAlign,
		IMG_UINT32 uiLog2HeapPageSize,
		DEVMEM_FLAGS_T uiFlags,
		const IMG_CHAR *pszText,
		DEVMEM_MEMDESC **ppsMemDescPtr)
{
	PVRSRV_ERROR eError;
	DEVMEM_MEMDESC *psMemDesc = NULL;
	DEVMEM_IMPORT *psImport;
	IMG_UINT32 ui32MappingTable = 0;

	eError = DevmemExportalignAdjustSizeAndAlign(uiLog2HeapPageSize,
			&uiSize,
			&uiAlign);
	if (eError != PVRSRV_OK)
	{
		goto failParams;
	}

	eError = _DevmemValidateParams(uiSize,
			uiAlign,
			&uiFlags);
	if (eError != PVRSRV_OK)
	{
		goto failParams;
	}

	eError =_DevmemMemDescAlloc(&psMemDesc);
	if (eError != PVRSRV_OK)
	{
		goto failMemDescAlloc;
	}

	eError = _AllocateDeviceMemory(hDevConnection,
			uiLog2HeapPageSize,
			uiSize,
			uiSize,
			1,
			1,
			&ui32MappingTable,
			uiAlign,
			uiFlags,
			IMG_TRUE,
			pszText,
			&psImport);
	if (eError != PVRSRV_OK)
	{
		goto failDeviceMemAlloc;
	}

	_DevmemMemDescInit(psMemDesc,
			0,
			psImport,
			uiSize);

	*ppsMemDescPtr = psMemDesc;

	/* copy the allocation descriptive name and size so it can be passed to DevicememHistory when
	 * the allocation gets mapped/unmapped
	 */
	_CheckAnnotationLength(pszText);
#if defined(__KERNEL__)
	OSStringLCopy(psMemDesc->szText, pszText, DEVMEM_ANNOTATION_MAX_LEN);
#else
	OSStringNCopy(psMemDesc->szText, pszText, DEVMEM_ANNOTATION_MAX_LEN);
	psMemDesc->szText[DEVMEM_ANNOTATION_MAX_LEN - 1] = '\0';
#endif	/* if defined(__KERNEL__) */


#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
		eError = BridgeRIWritePMREntry (GetBridgeHandle(psImport->hDevConnection),
				psImport->hPMR);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWritePMREntry failed (Error=%d)", __func__, eError));
		}

		/* Attach RI information */
		eError = BridgeRIWriteMEMDESCEntry (GetBridgeHandle(psImport->hDevConnection),
				psImport->hPMR,
				sizeof("^"),
				"^",
				psMemDesc->uiOffset,
				uiSize,
				IMG_FALSE,
				IMG_FALSE,
				&psMemDesc->hRIHandle);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWriteMEMDESCEntry failed (Error=%d)", __func__, eError));
		}
	}
#else  /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */
	PVR_UNREFERENCED_PARAMETER (pszText);
#endif /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */

	return PVRSRV_OK;

	/* error exit paths follow */

	failDeviceMemAlloc:
	_DevmemMemDescDiscard(psMemDesc);

	failMemDescAlloc:
	failParams:
	PVR_ASSERT(eError != PVRSRV_OK);
	PVR_DPF((PVR_DBG_ERROR,
			"%s: Failed! Error is %s. Allocation size: " IMG_DEVMEM_SIZE_FMTSPEC,
			__func__,
			PVRSRVGETERRORSTRING(eError),
			uiSize));
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAllocateSparse(SHARED_DEV_CONNECTION hDevConnection,
		IMG_DEVMEM_SIZE_T uiSize,
		IMG_DEVMEM_SIZE_T uiChunkSize,
		IMG_UINT32 ui32NumPhysChunks,
		IMG_UINT32 ui32NumVirtChunks,
		IMG_UINT32 *pui32MappingTable,
		IMG_DEVMEM_ALIGN_T uiAlign,
		IMG_UINT32 uiLog2HeapPageSize,
		DEVMEM_FLAGS_T uiFlags,
		const IMG_CHAR *pszText,
		DEVMEM_MEMDESC **ppsMemDescPtr)
{
	PVRSRV_ERROR eError;
	DEVMEM_MEMDESC *psMemDesc = NULL;
	DEVMEM_IMPORT *psImport;

	eError = DevmemExportalignAdjustSizeAndAlign(uiLog2HeapPageSize,
			&uiSize,
			&uiAlign);
	if (eError != PVRSRV_OK)
	{
		goto failParams;
	}

	eError = _DevmemValidateParams(uiSize,
			uiAlign,
			&uiFlags);
	if (eError != PVRSRV_OK)
	{
		goto failParams;
	}

	eError =_DevmemMemDescAlloc(&psMemDesc);
	if (eError != PVRSRV_OK)
	{
		goto failMemDescAlloc;
	}

	eError = _AllocateDeviceMemory(hDevConnection,
			uiLog2HeapPageSize,
			uiSize,
			uiChunkSize,
			ui32NumPhysChunks,
			ui32NumVirtChunks,
			pui32MappingTable,
			uiAlign,
			uiFlags,
			IMG_TRUE,
			pszText,
			&psImport);
	if (eError != PVRSRV_OK)
	{
		goto failDeviceMemAlloc;
	}

	_DevmemMemDescInit(psMemDesc,
			0,
			psImport,
			uiSize);

	/* copy the allocation descriptive name and size so it can be passed to DevicememHistory when
	 * the allocation gets mapped/unmapped
	 */
	_CheckAnnotationLength(pszText);
#if defined(__KERNEL__)
	OSStringLCopy(psMemDesc->szText, pszText, DEVMEM_ANNOTATION_MAX_LEN);
#else
	OSStringNCopy(psMemDesc->szText, pszText, DEVMEM_ANNOTATION_MAX_LEN);
	psMemDesc->szText[DEVMEM_ANNOTATION_MAX_LEN - 1] = '\0';
#endif	/* if defined(__KERNEL__) */


#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
		eError = BridgeRIWritePMREntry (GetBridgeHandle(psImport->hDevConnection),
				psImport->hPMR);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWritePMREntry failed (Error=%d)", __func__, eError));
		}

		/* Attach RI information */
		eError = BridgeRIWriteMEMDESCEntry (GetBridgeHandle(psMemDesc->psImport->hDevConnection),
				psMemDesc->psImport->hPMR,
				sizeof("^"),
				"^",
				psMemDesc->uiOffset,
				uiSize,
				IMG_FALSE,
				IMG_FALSE,
				&psMemDesc->hRIHandle);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWriteMEMDESCEntry failed (Error=%d)", __func__, eError));
		}
	}
#else  /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */
	PVR_UNREFERENCED_PARAMETER (pszText);
#endif /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */

	*ppsMemDescPtr = psMemDesc;

	return PVRSRV_OK;

	/* error exit paths follow */

	failDeviceMemAlloc:
	_DevmemMemDescDiscard(psMemDesc);

	failMemDescAlloc:
	failParams:
	PVR_ASSERT(eError != PVRSRV_OK);
	PVR_DPF((PVR_DBG_ERROR,
			"%s: Failed! Error is %s. Allocation size: " IMG_DEVMEM_SIZE_FMTSPEC,
			__func__,
			PVRSRVGETERRORSTRING(eError),
			uiSize));
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemMakeLocalImportHandle(SHARED_DEV_CONNECTION hDevConnection,
		IMG_HANDLE hServerHandle,
		IMG_HANDLE *hLocalImportHandle)
{
	return BridgePMRMakeLocalImportHandle(GetBridgeHandle(hDevConnection),
			hServerHandle,
			hLocalImportHandle);
}

IMG_INTERNAL PVRSRV_ERROR
DevmemUnmakeLocalImportHandle(SHARED_DEV_CONNECTION hDevConnection,
		IMG_HANDLE hLocalImportHandle)
{
	return DestroyServerResource(hDevConnection,
	                             NULL,
	                             BridgePMRUnmakeLocalImportHandle,
	                             hLocalImportHandle);
}

/*****************************************************************************
 *                Devmem unsecure export functions                           *
 *****************************************************************************/

#if defined(SUPPORT_INSECURE_EXPORT)

static PVRSRV_ERROR
_Mapping_Export(DEVMEM_IMPORT *psImport,
		DEVMEM_EXPORTHANDLE *phPMRExportHandlePtr,
		DEVMEM_EXPORTKEY *puiExportKeyPtr,
		DEVMEM_SIZE_T *puiSize,
		DEVMEM_LOG2ALIGN_T *puiLog2Contig)
{
	/* Gets an export handle and key for the PMR used for this mapping */
	/* Can only be done if there are no suballocations for this mapping */

	PVRSRV_ERROR eError;
	DEVMEM_EXPORTHANDLE hPMRExportHandle;
	DEVMEM_EXPORTKEY uiExportKey;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2Contig;

	if (psImport == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	if ((psImport->uiProperties & DEVMEM_PROPERTIES_EXPORTABLE) == 0)
	{
		eError = PVRSRV_ERROR_DEVICEMEM_CANT_EXPORT_SUBALLOCATION;
		goto failParams;
	}

	eError = BridgePMRExportPMR(GetBridgeHandle(psImport->hDevConnection),
			psImport->hPMR,
			&hPMRExportHandle,
			&uiSize,
			&uiLog2Contig,
			&uiExportKey);
	if (eError != PVRSRV_OK)
	{
		goto failExport;
	}

	PVR_ASSERT(uiSize == psImport->uiSize);

	*phPMRExportHandlePtr = hPMRExportHandle;
	*puiExportKeyPtr = uiExportKey;
	*puiSize = uiSize;
	*puiLog2Contig = uiLog2Contig;

	return PVRSRV_OK;

	/* error exit paths follow */

	failExport:
	failParams:

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;

}

static void
_Mapping_Unexport(DEVMEM_IMPORT *psImport,
		DEVMEM_EXPORTHANDLE hPMRExportHandle)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT (psImport != NULL);

	eError = DestroyServerResource(psImport->hDevConnection,
	                               NULL,
	                               BridgePMRUnexportPMR,
	                               hPMRExportHandle);
	PVR_ASSERT(eError == PVRSRV_OK);
}

IMG_INTERNAL PVRSRV_ERROR
DevmemExport(DEVMEM_MEMDESC *psMemDesc,
		DEVMEM_EXPORTCOOKIE *psExportCookie)
{
	/* Caller to provide storage for export cookie struct */
	PVRSRV_ERROR eError;
	IMG_HANDLE hPMRExportHandle = 0;
	IMG_UINT64 uiPMRExportPassword = 0;
	IMG_DEVMEM_SIZE_T uiSize = 0;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2Contig = 0;

	if (psMemDesc == NULL || psExportCookie == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	if (DEVMEM_PROPERTIES_EXPORTABLE !=
			(psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_EXPORTABLE))
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: This Memory (0x%p) cannot be exported!...",
				__func__, psMemDesc));
		eError = PVRSRV_ERROR_INVALID_REQUEST;
		goto e0;
	}

	eError = _Mapping_Export(psMemDesc->psImport,
			&hPMRExportHandle,
			&uiPMRExportPassword,
			&uiSize,
			&uiLog2Contig);
	if (eError != PVRSRV_OK)
	{
		psExportCookie->uiSize = 0;
		goto e0;
	}

	psExportCookie->hPMRExportHandle = hPMRExportHandle;
	psExportCookie->uiPMRExportPassword = uiPMRExportPassword;
	psExportCookie->uiSize = uiSize;
	psExportCookie->uiLog2ContiguityGuarantee = uiLog2Contig;

	return PVRSRV_OK;

	/* error exit paths follow */

	e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL void
DevmemUnexport(DEVMEM_MEMDESC *psMemDesc,
		DEVMEM_EXPORTCOOKIE *psExportCookie)
{
	_Mapping_Unexport(psMemDesc->psImport,
			psExportCookie->hPMRExportHandle);

	psExportCookie->uiSize = 0;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemImport(SHARED_DEV_CONNECTION hDevConnection,
		DEVMEM_EXPORTCOOKIE *psCookie,
		DEVMEM_FLAGS_T uiFlags,
		DEVMEM_MEMDESC **ppsMemDescPtr)
{
	DEVMEM_MEMDESC *psMemDesc = NULL;
	DEVMEM_IMPORT *psImport;
	IMG_HANDLE hPMR;
	PVRSRV_ERROR eError;

	if (ppsMemDescPtr == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	eError =_DevmemMemDescAlloc(&psMemDesc);
	if (eError != PVRSRV_OK)
	{
		goto failMemDescAlloc;
	}

	eError = _DevmemImportStructAlloc(hDevConnection,
			&psImport);
	if (eError != PVRSRV_OK)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto failImportAlloc;
	}

	/* Get a handle to the PMR (inc refcount) */
	eError = BridgePMRImportPMR(GetBridgeHandle(hDevConnection),
			psCookie->hPMRExportHandle,
			psCookie->uiPMRExportPassword,
			psCookie->uiSize, /* not trusted - just for sanity checks */
			psCookie->uiLog2ContiguityGuarantee, /* not trusted - just for sanity checks */
			&hPMR);
	if (eError != PVRSRV_OK)
	{
		goto failImport;
	}

	_DevmemImportStructInit(psImport,
			psCookie->uiSize,
			1ULL << psCookie->uiLog2ContiguityGuarantee,
			uiFlags,
			hPMR,
			DEVMEM_PROPERTIES_IMPORTED |
			DEVMEM_PROPERTIES_EXPORTABLE);

	_DevmemMemDescInit(psMemDesc,
			0,
			psImport,
			psImport->uiSize);

	*ppsMemDescPtr = psMemDesc;

#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psMemDesc->psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
		/* Attach RI information */
		eError = BridgeRIWriteMEMDESCEntry (GetBridgeHandle(psMemDesc->psImport->hDevConnection),
				psMemDesc->psImport->hPMR,
				sizeof("^"),
				"^",
				psMemDesc->uiOffset,
				psMemDesc->psImport->uiSize,
				IMG_TRUE,
				IMG_TRUE,
				&psMemDesc->hRIHandle);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWriteMEMDESCEntry failed (Error=%d)", __func__, eError));
		}
	}
#endif /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */

	return PVRSRV_OK;

	/* error exit paths follow */

	failImport:
	_DevmemImportDiscard(psImport);
	failImportAlloc:
	_DevmemMemDescDiscard(psMemDesc);
	failMemDescAlloc:
	failParams:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

#endif /* SUPPORT_INSECURE_EXPORT */

/*****************************************************************************
 *                   Common MemDesc functions                                *
 *****************************************************************************/
IMG_INTERNAL PVRSRV_ERROR
DevmemUnpin(DEVMEM_MEMDESC *psMemDesc)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	DEVMEM_IMPORT *psImport = psMemDesc->psImport;

	if (psImport->uiProperties & DEVMEM_PROPERTIES_NO_LAYOUT_CHANGE)
	{
		eError = PVRSRV_ERROR_INVALID_REQUEST;
		PVR_DPF((PVR_DBG_ERROR,
				"%s: The passed allocation is not valid to unpin",
				__func__));

		goto e_exit;
	}

	/* Stop if the allocation might have suballocations. */
	if (!(psImport->uiProperties & DEVMEM_PROPERTIES_EXPORTABLE))
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,
				"%s: The passed allocation is not valid to unpin because "
				"there might be suballocations on it. Make sure you allocate a page multiple "
				"of the heap when using PVRSRVAllocDeviceMem()",
				__func__));

		goto e_exit;
	}

	/* Stop if the Import is still mapped to CPU */
	if (psImport->sCPUImport.ui32RefCount)
	{
		eError = PVRSRV_ERROR_STILL_MAPPED;
		PVR_DPF((PVR_DBG_ERROR,
				"%s: There are still %u references on the CPU mapping. "
				"Please remove all CPU mappings before unpinning.",
				__func__,
				psImport->sCPUImport.ui32RefCount));

		goto e_exit;
	}

	/* Only unpin if it is not already unpinned
	 * Return PVRSRV_OK */
	if (psImport->uiProperties & DEVMEM_PROPERTIES_UNPINNED)
	{
		goto e_exit;
	}

	/* Unpin it and invalidate mapping */
	if (psImport->sDeviceImport.bMapped)
	{
		eError = BridgeDevmemIntUnpinInvalidate(GetBridgeHandle(psImport->hDevConnection),
				psImport->sDeviceImport.hMapping,
				psImport->hPMR);
	}
	else
	{
		/* Or just unpin it */
		eError = BridgeDevmemIntUnpin(GetBridgeHandle(psImport->hDevConnection),
				psImport->hPMR);
	}

	/* Update flags and RI when call was successful */
	if (eError == PVRSRV_OK)
	{
		psImport->uiProperties |= DEVMEM_PROPERTIES_UNPINNED;
	}
	else
	{
		/* Or just show what went wrong */
		PVR_DPF((PVR_DBG_ERROR, "%s: Unpin aborted because of error %d",
				__func__,
				eError));
	}

	e_exit:
	return eError;
}


IMG_INTERNAL PVRSRV_ERROR
DevmemPin(DEVMEM_MEMDESC *psMemDesc)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	DEVMEM_IMPORT *psImport = psMemDesc->psImport;

	if (psImport->uiProperties & DEVMEM_PROPERTIES_NO_LAYOUT_CHANGE)
	{
		eError = PVRSRV_ERROR_INVALID_REQUEST;
		goto e_exit;
	}

	/* Only pin if it is unpinned */
	if ((psImport->uiProperties & DEVMEM_PROPERTIES_UNPINNED) == 0)
	{
		goto e_exit;
	}

	/* Pin it and make mapping valid */
	if (psImport->sDeviceImport.bMapped)
	{
		eError = BridgeDevmemIntPinValidate(GetBridgeHandle(psImport->hDevConnection),
				psImport->sDeviceImport.hMapping,
				psImport->hPMR);
	}
	else
	{
		/* Or just pin it */
		eError = BridgeDevmemIntPin(GetBridgeHandle(psImport->hDevConnection),
				psImport->hPMR);
	}

	if ((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_PMR_NEW_MEMORY))
	{
		psImport->uiProperties &= ~DEVMEM_PROPERTIES_UNPINNED;
	}
	else
	{
		/* Or just show what went wrong */
		PVR_DPF((PVR_DBG_ERROR, "%s: Pin aborted because of error %d",
				__func__,
				eError));
	}

	e_exit:
	return eError;
}


IMG_INTERNAL PVRSRV_ERROR
DevmemGetSize(DEVMEM_MEMDESC *psMemDesc, IMG_DEVMEM_SIZE_T* puiSize)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	*puiSize = psMemDesc->uiAllocSize;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemGetAnnotation(DEVMEM_MEMDESC *psMemDesc, IMG_CHAR **pszAnnotation)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	*pszAnnotation = psMemDesc->szText;

	return eError;
}

/*
	This function is called for freeing any class of memory
 */
IMG_INTERNAL IMG_BOOL
DevmemFree(DEVMEM_MEMDESC *psMemDesc)
{
	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_SECURE)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Please use methods dedicated to secure buffers.",
				__func__));
		return IMG_FALSE;
	}

#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psMemDesc->psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
		if (psMemDesc->hRIHandle)
		{
			PVRSRV_ERROR eError;

			eError = BridgeRIDeleteMEMDESCEntry(GetBridgeHandle(psMemDesc->psImport->hDevConnection),
					psMemDesc->hRIHandle);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIDeleteMEMDESCEntry failed (Error=%d)", __func__, eError));
			}
		}
	}
#endif /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */

	return _DevmemMemDescRelease(psMemDesc);
}

IMG_INTERNAL PVRSRV_ERROR
DevmemMapToDevice(DEVMEM_MEMDESC *psMemDesc,
		DEVMEM_HEAP *psHeap,
		IMG_DEV_VIRTADDR *psDevVirtAddr)
{
	DEVMEM_IMPORT *psImport;
	IMG_DEV_VIRTADDR sDevVAddr;
	PVRSRV_ERROR eError;
	IMG_BOOL bMap = IMG_TRUE;
	IMG_BOOL bDestroyed = IMG_FALSE;

	/* Do not try to map unpinned memory */
	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_UNPINNED)
	{
		eError = PVRSRV_ERROR_INVALID_MAP_REQUEST;
		goto failFlags;
	}

	OSLockAcquire(psMemDesc->sDeviceMemDesc.hLock);
	if (psHeap == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	if (psMemDesc->sDeviceMemDesc.ui32RefCount != 0)
	{
		eError = PVRSRV_ERROR_DEVICEMEM_ALREADY_MAPPED;
		goto failCheck;
	}

	/* Don't map memory for deferred allocations */
	if (psMemDesc->psImport->uiFlags & PVRSRV_MEMALLOCFLAG_NO_OSPAGES_ON_ALLOC)
	{
		PVR_ASSERT(psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_EXPORTABLE);
		bMap = IMG_FALSE;
	}

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			__func__,
			psMemDesc,
			psMemDesc->sDeviceMemDesc.ui32RefCount,
			psMemDesc->sDeviceMemDesc.ui32RefCount+1);

	psImport = psMemDesc->psImport;
	_DevmemMemDescAcquire(psMemDesc);

	eError = _DevmemImportStructDevMap(psHeap,
			bMap,
			psImport,
			DEVICEMEM_UTILS_NO_ADDRESS);
	if (eError != PVRSRV_OK)
	{
		goto failMap;
	}

	sDevVAddr.uiAddr = psImport->sDeviceImport.sDevVAddr.uiAddr;
	sDevVAddr.uiAddr += psMemDesc->uiOffset;
	psMemDesc->sDeviceMemDesc.sDevVAddr = sDevVAddr;
	psMemDesc->sDeviceMemDesc.ui32RefCount++;

	*psDevVirtAddr = psMemDesc->sDeviceMemDesc.sDevVAddr;

	OSLockRelease(psMemDesc->sDeviceMemDesc.hLock);

	if (GetInfoPageDebugFlags(psMemDesc->psImport->hDevConnection) & DEBUG_FEATURE_PAGE_FAULT_DEBUG_ENABLED)
	{
		BridgeDevicememHistoryMap(GetBridgeHandle(psMemDesc->psImport->hDevConnection),
				psMemDesc->psImport->hPMR,
				psMemDesc->uiOffset,
				psMemDesc->sDeviceMemDesc.sDevVAddr,
				psMemDesc->uiAllocSize,
				psMemDesc->szText,
				DevmemGetHeapLog2PageSize(psHeap),
				psMemDesc->ui32AllocationIndex,
				&psMemDesc->ui32AllocationIndex);
	}

#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
		if (psMemDesc->hRIHandle)
		{
			eError = BridgeRIUpdateMEMDESCAddr(GetBridgeHandle(psImport->hDevConnection),
					psMemDesc->hRIHandle,
					psImport->sDeviceImport.sDevVAddr);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIUpdateMEMDESCAddr failed (Error=%d)", __func__, eError));
			}
		}
	}
#endif

	return PVRSRV_OK;

	failMap:
	bDestroyed = _DevmemMemDescRelease(psMemDesc);
	failCheck:
	failParams:
	if (!bDestroyed)
	{
		OSLockRelease(psMemDesc->sDeviceMemDesc.hLock);
	}
	PVR_ASSERT(eError != PVRSRV_OK);
	failFlags:
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemMapToDeviceAddress(DEVMEM_MEMDESC *psMemDesc,
		DEVMEM_HEAP *psHeap,
		IMG_DEV_VIRTADDR sDevVirtAddr)
{
	DEVMEM_IMPORT *psImport;
	IMG_DEV_VIRTADDR sDevVAddr;
	PVRSRV_ERROR eError;
	IMG_BOOL bMap = IMG_TRUE;
	IMG_BOOL bDestroyed = IMG_FALSE;

	/* Do not try to map unpinned memory */
	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_UNPINNED)
	{
		eError = PVRSRV_ERROR_INVALID_MAP_REQUEST;
		goto failFlags;
	}

	OSLockAcquire(psMemDesc->sDeviceMemDesc.hLock);
	if (psHeap == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	if (psMemDesc->sDeviceMemDesc.ui32RefCount != 0)
	{
		eError = PVRSRV_ERROR_DEVICEMEM_ALREADY_MAPPED;
		goto failCheck;
	}

	/* Don't map memory for deferred allocations */
	if (psMemDesc->psImport->uiFlags & PVRSRV_MEMALLOCFLAG_NO_OSPAGES_ON_ALLOC)
	{
		PVR_ASSERT(psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_EXPORTABLE);
		bMap = IMG_FALSE;
	}

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			__func__,
			psMemDesc,
			psMemDesc->sDeviceMemDesc.ui32RefCount,
			psMemDesc->sDeviceMemDesc.ui32RefCount+1);

	psImport = psMemDesc->psImport;
	_DevmemMemDescAcquire(psMemDesc);

	eError = _DevmemImportStructDevMap(psHeap,
			bMap,
			psImport,
			sDevVirtAddr.uiAddr);
	if (eError != PVRSRV_OK)
	{
		goto failMap;
	}

	sDevVAddr.uiAddr = psImport->sDeviceImport.sDevVAddr.uiAddr;
	sDevVAddr.uiAddr += psMemDesc->uiOffset;
	psMemDesc->sDeviceMemDesc.sDevVAddr = sDevVAddr;
	psMemDesc->sDeviceMemDesc.ui32RefCount++;

	OSLockRelease(psMemDesc->sDeviceMemDesc.hLock);

	if (GetInfoPageDebugFlags(psMemDesc->psImport->hDevConnection) & DEBUG_FEATURE_PAGE_FAULT_DEBUG_ENABLED)
	{
		BridgeDevicememHistoryMap(GetBridgeHandle(psMemDesc->psImport->hDevConnection),
				psMemDesc->psImport->hPMR,
				psMemDesc->uiOffset,
				psMemDesc->sDeviceMemDesc.sDevVAddr,
				psMemDesc->uiAllocSize,
				psMemDesc->szText,
				DevmemGetHeapLog2PageSize(psHeap),
				psMemDesc->ui32AllocationIndex,
				&psMemDesc->ui32AllocationIndex);
	}

#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
		if (psMemDesc->hRIHandle)
		{
			eError = BridgeRIUpdateMEMDESCAddr(GetBridgeHandle(psImport->hDevConnection),
					psMemDesc->hRIHandle,
					psImport->sDeviceImport.sDevVAddr);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIUpdateMEMDESCAddr failed (Error=%d)", __func__, eError));
			}
		}
	}
#endif

	return PVRSRV_OK;

	failMap:
	bDestroyed = _DevmemMemDescRelease(psMemDesc);
	failCheck:
	failParams:
	if (!bDestroyed)
	{
		OSLockRelease(psMemDesc->sDeviceMemDesc.hLock);
	}
	PVR_ASSERT(eError != PVRSRV_OK);
	failFlags:
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAcquireDevVirtAddr(DEVMEM_MEMDESC *psMemDesc,
		IMG_DEV_VIRTADDR *psDevVirtAddr)
{
	PVRSRV_ERROR eError;

	/* Do not try to map unpinned memory */
	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_UNPINNED)
	{
		eError = PVRSRV_ERROR_INVALID_MAP_REQUEST;
		goto failCheck;
	}

	OSLockAcquire(psMemDesc->sDeviceMemDesc.hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			__func__,
			psMemDesc,
			psMemDesc->sDeviceMemDesc.ui32RefCount,
			psMemDesc->sDeviceMemDesc.ui32RefCount+1);

	if (psMemDesc->sDeviceMemDesc.ui32RefCount == 0)
	{
		eError = PVRSRV_ERROR_DEVICEMEM_NO_MAPPING;
		goto failRelease;
	}
	psMemDesc->sDeviceMemDesc.ui32RefCount++;

	*psDevVirtAddr = psMemDesc->sDeviceMemDesc.sDevVAddr;
	OSLockRelease(psMemDesc->sDeviceMemDesc.hLock);

	return PVRSRV_OK;

	failRelease:
	OSLockRelease(psMemDesc->sDeviceMemDesc.hLock);
	PVR_ASSERT(eError != PVRSRV_OK);
	failCheck:
	return eError;
}

IMG_INTERNAL void
DevmemReleaseDevVirtAddr(DEVMEM_MEMDESC *psMemDesc)
{
	PVR_ASSERT(psMemDesc != NULL);

	OSLockAcquire(psMemDesc->sDeviceMemDesc.hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			__func__,
			psMemDesc,
			psMemDesc->sDeviceMemDesc.ui32RefCount,
			psMemDesc->sDeviceMemDesc.ui32RefCount-1);

	PVR_ASSERT(psMemDesc->sDeviceMemDesc.ui32RefCount != 0);

	if (--psMemDesc->sDeviceMemDesc.ui32RefCount == 0)
	{
		if (GetInfoPageDebugFlags(psMemDesc->psImport->hDevConnection) & DEBUG_FEATURE_PAGE_FAULT_DEBUG_ENABLED)
		{
			BridgeDevicememHistoryUnmap(GetBridgeHandle(psMemDesc->psImport->hDevConnection),
					psMemDesc->psImport->hPMR,
					psMemDesc->uiOffset,
					psMemDesc->sDeviceMemDesc.sDevVAddr,
					psMemDesc->uiAllocSize,
					psMemDesc->szText,
					DevmemGetHeapLog2PageSize(psMemDesc->psImport->sDeviceImport.psHeap),
					psMemDesc->ui32AllocationIndex,
					&psMemDesc->ui32AllocationIndex);
		}

		_DevmemImportStructDevUnmap(psMemDesc->psImport);
		OSLockRelease(psMemDesc->sDeviceMemDesc.hLock);

		_DevmemMemDescRelease(psMemDesc);
	}
	else
	{
		OSLockRelease(psMemDesc->sDeviceMemDesc.hLock);
	}
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAcquireCpuVirtAddr(DEVMEM_MEMDESC *psMemDesc,
		void **ppvCpuVirtAddr)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT(psMemDesc != NULL);
	PVR_ASSERT(ppvCpuVirtAddr != NULL);

	if (psMemDesc->psImport->uiProperties &
			(DEVMEM_PROPERTIES_UNPINNED | DEVMEM_PROPERTIES_SECURE))
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Allocation is currently unpinned or a secure buffer. "
				"Not possible to map to CPU!",
				__func__));
		eError = PVRSRV_ERROR_INVALID_MAP_REQUEST;
		goto failFlags;
	}

	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_NO_CPU_MAPPING)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: CPU Mapping is not possible on this allocation!",
				__func__));
		eError = PVRSRV_ERROR_INVALID_MAP_REQUEST;
		goto failFlags;
	}

	OSLockAcquire(psMemDesc->sCPUMemDesc.hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			__func__,
			psMemDesc,
			psMemDesc->sCPUMemDesc.ui32RefCount,
			psMemDesc->sCPUMemDesc.ui32RefCount+1);

	if (psMemDesc->sCPUMemDesc.ui32RefCount++ == 0)
	{
		DEVMEM_IMPORT *psImport = psMemDesc->psImport;
		IMG_UINT8 *pui8CPUVAddr;

		_DevmemMemDescAcquire(psMemDesc);
		eError = _DevmemImportStructCPUMap(psImport);
		if (eError != PVRSRV_OK)
		{
			goto failMap;
		}

		pui8CPUVAddr = psImport->sCPUImport.pvCPUVAddr;
		pui8CPUVAddr += psMemDesc->uiOffset;
		psMemDesc->sCPUMemDesc.pvCPUVAddr = pui8CPUVAddr;
	}
	*ppvCpuVirtAddr = psMemDesc->sCPUMemDesc.pvCPUVAddr;

	VG_MARK_INITIALIZED(*ppvCpuVirtAddr, psMemDesc->psImport->uiSize);

	OSLockRelease(psMemDesc->sCPUMemDesc.hLock);

	return PVRSRV_OK;

	failMap:
	PVR_ASSERT(eError != PVRSRV_OK);
	psMemDesc->sCPUMemDesc.ui32RefCount--;

	if (!_DevmemMemDescRelease(psMemDesc))
	{
		OSLockRelease(psMemDesc->sCPUMemDesc.hLock);
	}
	failFlags:
	return eError;
}

IMG_INTERNAL void
DevmemReacquireCpuVirtAddr(DEVMEM_MEMDESC *psMemDesc,
		void **ppvCpuVirtAddr)
{
	PVR_ASSERT(psMemDesc != NULL);
	PVR_ASSERT(ppvCpuVirtAddr != NULL);

	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_NO_CPU_MAPPING)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: CPU UnMapping is not possible on this allocation!",
				__func__));
		return;
	}

	OSLockAcquire(psMemDesc->sCPUMemDesc.hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			__func__,
			psMemDesc,
			psMemDesc->sCPUMemDesc.ui32RefCount,
			psMemDesc->sCPUMemDesc.ui32RefCount+1);

	*ppvCpuVirtAddr = NULL;
	if (psMemDesc->sCPUMemDesc.ui32RefCount)
	{
		*ppvCpuVirtAddr = psMemDesc->sCPUMemDesc.pvCPUVAddr;
		psMemDesc->sCPUMemDesc.ui32RefCount += 1;
	}

	VG_MARK_INITIALIZED(*ppvCpuVirtAddr, psMemDesc->psImport->uiSize);
	OSLockRelease(psMemDesc->sCPUMemDesc.hLock);
}

IMG_INTERNAL void
DevmemReleaseCpuVirtAddr(DEVMEM_MEMDESC *psMemDesc)
{
	PVR_ASSERT(psMemDesc != NULL);

	if (psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_NO_CPU_MAPPING)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: CPU UnMapping is not possible on this allocation!",
				__func__));
		return;
	}

	OSLockAcquire(psMemDesc->sCPUMemDesc.hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			__func__,
			psMemDesc,
			psMemDesc->sCPUMemDesc.ui32RefCount,
			psMemDesc->sCPUMemDesc.ui32RefCount-1);

	PVR_ASSERT(psMemDesc->sCPUMemDesc.ui32RefCount != 0);

	if (--psMemDesc->sCPUMemDesc.ui32RefCount == 0)
	{
		OSLockRelease(psMemDesc->sCPUMemDesc.hLock);
		_DevmemImportStructCPUUnmap(psMemDesc->psImport);
		_DevmemMemDescRelease(psMemDesc);
	}
	else
	{
		OSLockRelease(psMemDesc->sCPUMemDesc.hLock);
	}
}

IMG_INTERNAL PVRSRV_ERROR
DevmemLocalGetImportHandle(DEVMEM_MEMDESC *psMemDesc,
		IMG_HANDLE *phImport)
{
	if ((psMemDesc->psImport->uiProperties & DEVMEM_PROPERTIES_EXPORTABLE) == 0)
	{
		return PVRSRV_ERROR_DEVICEMEM_CANT_EXPORT_SUBALLOCATION;
	}

	*phImport = psMemDesc->psImport->hPMR;

	return PVRSRV_OK;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemGetImportUID(DEVMEM_MEMDESC *psMemDesc,
		IMG_UINT64 *pui64UID)
{
	DEVMEM_IMPORT *psImport = psMemDesc->psImport;
	PVRSRV_ERROR eError;

	if (!(psImport->uiProperties & (DEVMEM_PROPERTIES_IMPORTED |
		                        DEVMEM_PROPERTIES_EXPORTABLE)))
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: This Memory (0x%p) doesn't support the functionality requested...",
				__func__, psMemDesc));
		return PVRSRV_ERROR_INVALID_REQUEST;
	}

	eError = BridgePMRGetUID(GetBridgeHandle(psImport->hDevConnection),
			psImport->hPMR,
			pui64UID);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemGetReservation(DEVMEM_MEMDESC *psMemDesc,
		IMG_HANDLE *hReservation)
{
	DEVMEM_IMPORT *psImport;

	PVR_ASSERT(psMemDesc);
	psImport = psMemDesc->psImport;

	PVR_ASSERT(psImport);
	*hReservation = psImport->sDeviceImport.hReservation;

	return PVRSRV_OK;
}

PVRSRV_ERROR
DevmemGetPMRData(DEVMEM_MEMDESC *psMemDesc,
		IMG_HANDLE *phPMR,
		IMG_DEVMEM_OFFSET_T *puiPMROffset)
{
	DEVMEM_IMPORT *psImport;

	PVR_ASSERT(psMemDesc);
	*puiPMROffset = psMemDesc->uiOffset;
	psImport = psMemDesc->psImport;

	PVR_ASSERT(psImport);
	*phPMR = psImport->hPMR;

	return PVRSRV_OK;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemGetFlags(DEVMEM_MEMDESC *psMemDesc,
		DEVMEM_FLAGS_T *puiFlags)
{
	DEVMEM_IMPORT *psImport;

	PVR_ASSERT(psMemDesc);
	psImport = psMemDesc->psImport;

	PVR_ASSERT(psImport);
	*puiFlags = psImport->uiFlags;

	return PVRSRV_OK;
}

IMG_INTERNAL SHARED_DEV_CONNECTION
DevmemGetConnection(DEVMEM_MEMDESC *psMemDesc)
{
	return psMemDesc->psImport->hDevConnection;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemLocalImport(SHARED_DEV_CONNECTION hDevConnection,
		IMG_HANDLE hExtHandle,
		DEVMEM_FLAGS_T uiFlags,
		DEVMEM_MEMDESC **ppsMemDescPtr,
		IMG_DEVMEM_SIZE_T *puiSizePtr,
		const IMG_CHAR *pszAnnotation)
{
	DEVMEM_MEMDESC *psMemDesc = NULL;
	DEVMEM_IMPORT *psImport;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_DEVMEM_ALIGN_T uiAlign;
	IMG_HANDLE hPMR;
	PVRSRV_ERROR eError;

	if (ppsMemDescPtr == NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	eError =_DevmemMemDescAlloc(&psMemDesc);
	if (eError != PVRSRV_OK)
	{
		goto failMemDescAlloc;
	}

	eError = _DevmemImportStructAlloc(hDevConnection,
			&psImport);
	if (eError != PVRSRV_OK)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto failImportAlloc;
	}

	/* Get the PMR handle and its size from the server */
	eError = BridgePMRLocalImportPMR(GetBridgeHandle(hDevConnection),
			hExtHandle,
			&hPMR,
			&uiSize,
			&uiAlign);
	if (eError != PVRSRV_OK)
	{
		goto failImport;
	}

	_DevmemImportStructInit(psImport,
			uiSize,
			uiAlign,
			uiFlags,
			hPMR,
			DEVMEM_PROPERTIES_IMPORTED |
			DEVMEM_PROPERTIES_EXPORTABLE);

	_DevmemMemDescInit(psMemDesc,
			0,
			psImport,
			uiSize);

	*ppsMemDescPtr = psMemDesc;
	if (puiSizePtr)
		*puiSizePtr = uiSize;

#if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO)
	if (PVRSRVIsBridgeEnabled(GetBridgeHandle(psMemDesc->psImport->hDevConnection), PVRSRV_BRIDGE_RI))
	{
		/* Attach RI information.
		 * Set backed size to 0 since this allocation has been allocated
		 * by the same process and has been accounted for. */
		eError = BridgeRIWriteMEMDESCEntry (GetBridgeHandle(psMemDesc->psImport->hDevConnection),
				psMemDesc->psImport->hPMR,
				sizeof("^"),
				"^",
				psMemDesc->uiOffset,
				psMemDesc->psImport->uiSize,
				IMG_TRUE,
				IMG_FALSE,
				&(psMemDesc->hRIHandle));
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: call to BridgeRIWriteMEMDESCEntry failed (Error=%d)", __func__, eError));
		}
	}
#endif /* if defined(PVRSRV_ENABLE_GPU_MEMORY_INFO) */


	/* Copy the allocation descriptive name and size so it can be passed
	 * to DevicememHistory when the allocation gets mapped/unmapped
	 */
	_CheckAnnotationLength(pszAnnotation);
#if defined(__KERNEL__)
	OSStringLCopy(psMemDesc->szText, pszAnnotation, DEVMEM_ANNOTATION_MAX_LEN);
#else
	OSStringNCopy(psMemDesc->szText, pszAnnotation, DEVMEM_ANNOTATION_MAX_LEN);
	psMemDesc->szText[DEVMEM_ANNOTATION_MAX_LEN - 1] = '\0';
#endif	/* if defined(__KERNEL__) */

	return PVRSRV_OK;

	failImport:
	_DevmemImportDiscard(psImport);
	failImportAlloc:
	_DevmemMemDescDiscard(psMemDesc);
	failMemDescAlloc:
	failParams:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemIsDevVirtAddrValid(DEVMEM_CONTEXT *psContext,
		IMG_DEV_VIRTADDR sDevVAddr)
{
	return BridgeDevmemIsVDevAddrValid(GetBridgeHandle(psContext->hDevConnection),
			psContext->hDevMemServerContext,
			sDevVAddr);
}


IMG_INTERNAL PVRSRV_ERROR
DevmemGetFaultAddress(DEVMEM_CONTEXT *psContext,
		IMG_DEV_VIRTADDR *psFaultAddress)
{
	return BridgeDevmemGetFaultAddress(GetBridgeHandle(psContext->hDevConnection),
			psContext->hDevMemServerContext,
			psFaultAddress);
}

IMG_INTERNAL IMG_UINT32
DevmemGetHeapLog2PageSize(DEVMEM_HEAP *psHeap)
{
	return psHeap->uiLog2Quantum;
}

IMG_INTERNAL IMG_UINT32
DevmemGetHeapTilingProperties(DEVMEM_HEAP *psHeap,
		IMG_UINT32 *puiLog2ImportAlignment,
		IMG_UINT32 *puiLog2TilingStrideFactor)
{
	*puiLog2ImportAlignment = psHeap->uiLog2ImportAlignment;
	*puiLog2TilingStrideFactor = psHeap->uiLog2TilingStrideFactor;
	return PVRSRV_OK;
}

/**************************************************************************/ /*!
@Function       RegisterDevMemPFNotify
@Description    Registers that the application wants to be signaled when a page
                fault occurs.

@Input          psContext      Memory context the process that would like to
                               be notified about.
@Input          ui32PID        The PID of the calling process.
@Input          bRegister      If true, register. If false, de-register.
@Return         PVRSRV_ERROR:  PVRSRV_OK on success. Otherwise, a PVRSRV_
                               error code
 */ /***************************************************************************/
IMG_INTERNAL PVRSRV_ERROR
RegisterDevmemPFNotify(DEVMEM_CONTEXT *psContext,
		IMG_UINT32     ui32PID,
		IMG_BOOL       bRegister)
{
	PVRSRV_ERROR eError;

	eError = BridgeDevmemIntRegisterPFNotifyKM(GetBridgeHandle(psContext->hDevConnection),
			psContext->hDevMemServerContext,
			ui32PID,
			bRegister);
	if (eError == PVRSRV_ERROR_BRIDGE_CALL_FAILED)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Bridge Call Failed: This could suggest a UM/KM mismatch (%d)",
				__func__,
				(IMG_INT)(eError)));
	}

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
GetMaxDevMemSize(SHARED_DEV_CONNECTION hDevConnection,
		IMG_DEVMEM_SIZE_T *puiLMASize,
		IMG_DEVMEM_SIZE_T *puiUMASize)
{
	return BridgeGetMaxDevMemSize(GetBridgeHandle(hDevConnection),
			puiLMASize,
			puiUMASize);
}
