/*
 * sbusWrapperC.cpp
 *
 *  Created on: May 25, 2025
 *      Author: louis
 */

// Includes from project
#include "Radio/sbusWrapperC.h"
#include "main.h"

/*
 * ============================================================================
 * DMA Buffer and Cache Coherency - Important Information
 * ============================================================================
 *
 * WHY THIS BUFFER NEEDS SPECIAL HANDLING:
 *
 * 1. MEMORY LOCATION (.axisram_bss section):
 *    - The STM32H7 has multiple RAM regions: DTCM, AXI-SRAM (D1), SRAM (D2), etc.
 *    - DMA controllers CANNOT access DTCM RAM (the default location for .bss)
 *    - We place this buffer in AXI-SRAM (D2 domain) which is DMA-accessible
 *
 * 2. THE CACHE COHERENCY PROBLEM:
 *    - The STM32H7 Cortex-M7 has a Data Cache (D-Cache) to speed up CPU memory access
 *    - When the CPU reads memory, it first checks the cache (very fast)
 *    - If data is in cache, it reads from there instead of going to RAM
 *
 *    THE ISSUE WITH DMA:
 *    - DMA writes data directly to RAM, bypassing the CPU cache
 *    - The CPU cache still contains old/stale data
 *    - When we read sbusBuf, the CPU reads OLD data from cache, not the NEW data from DMA!
 *    - Result: We see garbage/unchanging data even though DMA is working correctly
 *
 * 3. WHY 32-BYTE ALIGNMENT:
 *    - The D-Cache is organized in "cache lines" of 32 bytes each
 *    - Cache operations (like invalidation) work on entire cache lines, not individual bytes
 *    - If the buffer is not aligned to 32 bytes, invalidating it might:
 *      a) Miss some cache lines containing our data
 *      b) Corrupt adjacent data by invalidating their cache lines
 *    - By aligning to 32 bytes and sizing to a multiple of 32, we ensure clean cache operations
 *
 * 4. THE SOLUTION:
 *    - Before reading the DMA buffer, we call SCB_InvalidateDCache_by_Addr()
 *    - This tells the CPU: "throw away cached data for this memory region"
 *    - Next CPU read will fetch fresh data from RAM (where DMA wrote it)
 *    - The 32-byte alignment ensures this operation is safe and efficient
 *
 * SUMMARY: DMA writes to RAM → Cache still has old data → We invalidate cache →
 *          CPU reads fresh data from RAM ✓
 */

// Place the SBUS DMA receive buffer into AXI-SRAM (D2) so the DMA engine can write to it.
// The default DTCM section is not accessible by DMA, causing the buffer to remain zeroed.
// IMPORTANT: Must be 32-byte aligned for proper D-Cache invalidation (see comment above)
uint8_t sbusBuf[32] __attribute__((aligned(32))) __attribute__((section(".axisram_bss")));
uint8_t sbusBufCopy[SBUS_FRAME_SIZE];
bool g_newFrameReady = false;


/*
 * Copy the received buffer from DMA to a global variable
 *
 * This function is called from the UART IDLE interrupt, indicating a complete
 * SBUS frame has been received by DMA into sbusBuf.
 */
void copyFrame()
{
	// CRITICAL: Invalidate D-Cache before reading the DMA buffer
	// This forces the CPU to read fresh data from RAM instead of stale cached data
	// (See detailed explanation in the comment block above sbusBuf declaration)
	SCB_InvalidateDCache_by_Addr((uint32_t*)sbusBuf, SBUS_FRAME_SIZE);

	for (int i = 0; i < SBUS_FRAME_SIZE; ++i)
	{
		sbusBufCopy[i] = sbusBuf[i];
	}

	g_newFrameReady = true;
}

