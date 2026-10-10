package main

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"os"
	"sort"
	"strconv"
	"strings"
	"time"
)

// captureFormatter (investigation only) dumps the FULL machine state at the current
// PC to fmt_snapshot.bin, then single-steps `k` instructions PURELY (no MMIO tick, no
// IRQ — interrupts cleared) logging the architectural state before each step to
// fmt_trace_go.txt. The Python oracle (re/oracle_diff.py) loads the snapshot, steps
// the SAME way, and diffs line-by-line; the first divergence pins the mis-emulated
// instruction in the BCD→glyph formatter (RECON_NOTES cont.17c). Harness-only.
func captureFormatter(cpu *CPU, mem *Memory, k int) {
	// ---- snapshot: 36 regs (LE u32) then dram, ilram, ocram raw ----
	regs := []uint32{}
	regs = append(regs, cpu.r[:]...)
	regs = append(regs, cpu.rbank1[:]...)
	regs = append(regs, cpu.pc, cpu.pr, cpu.gbr, cpu.vbr, cpu.ssr, cpu.spc, cpu.sgr,
		cpu.mach, cpu.macl, cpu.fpul, cpu.fpscr, cpu.sr)
	snap, err := os.Create("fmt_snapshot.bin")
	if err != nil {
		fmt.Println("snapshot create error:", err)
		return
	}
	binary.Write(snap, binary.LittleEndian, regs)
	snap.Write(mem.dram)
	snap.Write(mem.ilram)
	snap.Write(mem.ocram)
	snap.Close()
	fmt.Printf("=== captureFormatter: snapshot @pc=0x%08x r4=0x%08x ([r4]=0x%08x), %d regs + %dB dram + %dB ilram + %dB ocram ===\n",
		cpu.pc, cpu.r[4], mem.R32(cpu.r[4]), len(regs), len(mem.dram), len(mem.ilram), len(mem.ocram))

	// ---- pure lockstep trace (mirror of step() minus acceptInterrupt) ----
	cpu.pending = nil
	tf, err := os.Create("fmt_trace_go.txt")
	if err != nil {
		fmt.Println("trace create error:", err)
		return
	}
	w := bufio.NewWriterSize(tf, 1<<20)
	for i := 0; i < k; i++ {
		fmt.Fprintf(w, "%d %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
			i, cpu.pc,
			cpu.r[0], cpu.r[1], cpu.r[2], cpu.r[3], cpu.r[4], cpu.r[5], cpu.r[6], cpu.r[7],
			cpu.r[8], cpu.r[9], cpu.r[10], cpu.r[11], cpu.r[12], cpu.r[13], cpu.r[14], cpu.r[15],
			cpu.sr, cpu.mach, cpu.macl, cpu.pr, cpu.gbr)
		op := mem.R16(cpu.pc)
		cpu.pc += 2
		cpu.execute(op)
		cpu.cycles++
	}
	w.Flush()
	tf.Close()
	fmt.Printf("=== captureFormatter: traced %d steps -> fmt_trace_go.txt (final pc=0x%08x) ===\n", k, cpu.pc)
}

// dumpFB decodes a 396x224 RGB565 (big-endian) framebuffer at the given cached
// DRAM address and writes it as a PNG, so we can SEE what the emulator displays.
func dumpFB(mem *Memory, cached uint32, path string) {
	const W, H = 384, 216 // fx-CG50 framebuffer: 384px stride (usable area in the 396x224 panel)
	img := image.NewRGBA(image.Rect(0, 0, W, H))
	for y := 0; y < H; y++ {
		for x := 0; x < W; x++ {
			p := mem.R16(cached + uint32(y*W+x)*2)
			r := uint8((p>>11)&0x1F) << 3
			g := uint8((p>>5)&0x3F) << 2
			b := uint8(p&0x1F) << 3
			img.Set(x, y, color.RGBA{r, g, b, 255})
		}
	}
	f, err := os.Create(path)
	if err != nil {
		fmt.Println("dumpFB:", err)
		return
	}
	defer f.Close()
	png.Encode(f, img)
	fmt.Printf("wrote %s (%dx%d from 0x%08x)\n", path, W, H, cached)
}

// Runner: boot the 3.60 full flash and run, optionally with the periodic timer.
// Usage: go run . [maxIns] [timerPeriod]   (timerPeriod 0 = no interrupts)
func main() {
	if len(os.Args) > 1 && os.Args[1] == "linux" {
		runLinux(os.Args[2:])
		return
	}
	maxIns := uint64(2_000_000)
	var timerPeriod uint64 = 0
	if len(os.Args) > 1 {
		maxIns = parseUint(os.Args[1])
	}
	if len(os.Args) > 2 {
		timerPeriod = parseUint(os.Args[2])
	}

	img, err := os.ReadFile(bootImagePath)
	if err != nil {
		fmt.Println("error:", err)
		return
	}
	mmio := NewMMIOBus()
	mem := NewMemory(img, mmio)
	cpu := NewCPU(mem)
	mmio.cpu = cpu // wire the cpu back-ref so cycle-based MMIO (ETMU down-counter
	mem.cpu = cpu  // wire cpu back-ref for the optional DRAM read-watch (investigation)
	// @0xA44D00D8) advances; without it FUN_803742f8's busy-delay never completes.
	mmio.watchPC = map[uint32]int{}
	mmio.watchBase = 0xA4080000 // attribute reads of this region to the reader PC (KEYSC scan)
	cpu.pc = 0x80000000
	if os.Getenv("CG50_WARM") != "" {
		// Power on from "off" instead of from a dead battery: the OS's power-off paths set this
		// IL RAM word to 1, and the boot (FUN_80365238) skips the first-boot setup wizard only
		// when it is 1. Needs a flash image whose fls0 is already set up (the 32 MB dump).
		mem.W32(warmFlagAddr, 1)
	}
	mmio.timerPeriod = timerPeriod
	mmio.timerNext = 0

	mode := ""
	if len(os.Args) > 3 {
		mode = os.Args[3]
	}

	// Persistence (so the Android app doesn't redo first-boot setup every launch). A real
	// already-set-up calc keeps its config in NOR-flash fls0 PLUS battery-backed RAM; the
	// fls0 mount is coupled to that RAM, so flash-only persistence still triggers the setup
	// wizard (fls0_open -> -6 -> format -> language screen, confirmed in Ghidra: FUN_80365238).
	// So we persist a FULL save-state (CPU + DRAM/ILRAM/OCRAM + flash delta) and RESUME from
	// it. "provision" boots fresh, drives setup to the MAIN MENU, and snapshots; every other
	// boot auto-resumes from the snapshot. File lives under os/ (git-ignored; OS-derived bytes).
	resumed := false
	if mode == "provision" {
		seqStr := "1-9,1-9,1-9,1-9,6-9,6-9,1-9,1-9,2-1" // first-boot setup -> MAIN MENU
		pressAt := uint64(130_000_000)
		interval := uint64(14_000_000)
		if len(os.Args) > 4 && os.Args[4] != "" {
			seqStr = os.Args[4]
		}
		if len(os.Args) > 5 {
			pressAt = parseUint(os.Args[5])
		}
		if len(os.Args) > 6 {
			interval = parseUint(os.Args[6])
		}
		runSeq(cpu, mem, mmio, maxIns, seqStr, pressAt, interval, false)
		if err := SaveState(statePath, cpu, mem); err != nil {
			fmt.Println("save-state: save error:", err)
		} else {
			fmt.Printf("save-state: snapshot written to %s (pc=0x%08x) — future boots resume here\n", statePath, cpu.pc)
		}
		return
	}
	if _, statErr := os.Stat(statePath); statErr == nil {
		if err := LoadState(statePath, cpu, mem); err != nil {
			fmt.Println("save-state: load failed, booting fresh:", err)
		} else {
			cpu.cycles = 0 // restart the free-running counter (OS uses timer deltas only)
			mmio.timerNext = mmio.timerPeriod
			mmio.timerTicks = 0
			mmio.keysc.resumeDefaults()
			resumed = true
			fmt.Printf("save-state: RESUMED from %s at pc=0x%08x (skipped boot + first-setup)\n", statePath, cpu.pc)
		}
	}

	if mode == "scancap" {
		runScanCap(cpu, mem, mmio)
		return
	}
	if mode == "web" {
		runWeb(cpu, mem, mmio, resumed)
		return
	}
	if mode == "rtbench" {
		// Demonstrate/measure the real-time paced loop via the Emulator facade (the same API
		// the Android bridge drives). Resumes from the save-state if present, then paces to a
		// target IPS for ~3s and reports the achieved rate + frame count.
		runRTBench(img)
		return
	}
	if mode == "ftl" {
		runFTL(cpu, mem, mmio, maxIns)
		return
	}
	if mode == "stack" {
		runStack(cpu, mem, mmio, maxIns)
		return
	}
	if mode == "gate" {
		runGate(cpu, mem, mmio, maxIns)
		return
	}
	if mode == "draw" {
		runDraw(cpu, mem, mmio, maxIns)
		return
	}
	if mode == "shelltrace" {
		runShellTrace(cpu, mem, mmio, maxIns)
		return
	}
	if mode == "wmap" {
		runWmap(cpu, mem, mmio, maxIns)
		return
	}
	if mode == "flashwr" {
		runFlashWr(cpu, mem, mmio, maxIns)
		return
	}
	if mode == "drive" {
		// args: [4]=kbReg(hex, -1=all) [5]=kbVal(hex) [6]=pressAtCycle(dec)
		reg := int64(-1)
		val := uint64(0xFFFF)
		pressAt := uint64(110_000_000)
		if len(os.Args) > 4 {
			reg = int64(int32(parseUint(os.Args[4])))
		}
		if len(os.Args) > 5 {
			val = parseUint(os.Args[5])
		}
		if len(os.Args) > 6 {
			pressAt = parseUint(os.Args[6])
		}
		mmio.kbReg = int32(reg)
		mmio.kbVal = uint32(val)
		mmio.kbStart = pressAt
		mmio.kbEnd = pressAt + 3_000_000 // hold ~3M instr
		runDrive(cpu, mem, mmio, maxIns)
		return
	}
	if mode == "key" {
		// args: [4]=row(dec) [5]=col(dec) [6]=pressAtCycle(dec)
		row, col := uint32(5), uint32(3) // default: a mid-matrix key
		pressAt := uint64(160_000_000)
		if len(os.Args) > 4 {
			row = uint32(parseUint(os.Args[4]))
		}
		if len(os.Args) > 5 {
			col = uint32(parseUint(os.Args[5]))
		}
		if len(os.Args) > 6 {
			pressAt = parseUint(os.Args[6])
		}
		runKey(cpu, mem, mmio, maxIns, row, col, pressAt)
		return
	}
	if mode == "seq" {
		// args: [4]="row-col,row-col,..." [5]=pressAt(dec) [6]=interval(dec)
		seqStr := "1-9" // default: one 'advance' (C10R2 = code 0x29)
		pressAt := uint64(130_000_000)
		interval := uint64(8_000_000)
		if len(os.Args) > 4 {
			seqStr = os.Args[4]
		}
		if len(os.Args) > 5 {
			pressAt = parseUint(os.Args[5])
		}
		if len(os.Args) > 6 {
			interval = parseUint(os.Args[6])
		}
		runSeq(cpu, mem, mmio, maxIns, seqStr, pressAt, interval, false)
		return
	}
	if mode == "bcdseq" {
		// Same driver as "seq" but with the BCD operand read-watch armed: scans DRAM
		// for the typed literal's BCD bytes, locks a watch window on it, and histograms
		// the PCs that READ it during EXE eval. args identical to "seq".
		seqStr := "1-9"
		pressAt := uint64(130_000_000)
		interval := uint64(8_000_000)
		if len(os.Args) > 4 {
			seqStr = os.Args[4]
		}
		if len(os.Args) > 5 {
			pressAt = parseUint(os.Args[5])
		}
		if len(os.Args) > 6 {
			interval = parseUint(os.Args[6])
		}
		runSeq(cpu, mem, mmio, maxIns, seqStr, pressAt, interval, true)
		return
	}
	if mode == "prof" {
		runProf(cpu, mem, mmio, maxIns)
		return
	}
	fmt.Printf("booting flash_full.bin (%d bytes); maxIns=%d timer=%d\n", len(img), maxIns, timerPeriod)
	start := time.Now()
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x after %d instr: %v\n", cpu.pc, cpu.cycles, r)
			report(cpu, mmio, start)
		}
	}()

	checkpoint := maxIns / 20
	if checkpoint == 0 {
		checkpoint = 1
	}
	seen := map[uint32]bool{}
	bestNZ := 0
	vramNZ := func() (uint32, int) {
		sar, _ := mmio.FrameSAR()
		if sar == 0 {
			sar = 0x0C000000
		}
		vram := 0x8C000000 | (sar & 0x1FFFFFFF)
		nz := 0
		for k := uint32(0); k < 396*224; k += 8 {
			if mem.R16(vram+k*2) != 0 {
				nz++
			}
		}
		return vram, nz * 8
	}
	for cpu.cycles < maxIns {
		mmio.tick(cpu)
		cpu.step()
		if cpu.cycles&0x3FFFF == 0 { // ~every 256k: watch for new frame pushes
			if sar, ok := mmio.FrameSAR(); ok && !seen[sar] {
				seen[sar] = true
				fmt.Printf("  [%10d] VRAM->LCD push SAR=0x%08x\n", cpu.cycles, sar)
			}
		}
		if cpu.cycles%checkpoint == 0 {
			vram, nz := vramNZ()
			if nz > bestNZ {
				bestNZ = nz
			}
			fmt.Printf("  [%10d] PC=0x%08x SR=0x%08x ticks=%d irqs=%d vram_nz~%d (vram=0x%08x)\n",
				cpu.cycles, cpu.pc, cpu.sr, mmio.timerTicks, cpu.irqCnt, nz, vram)
		}
	}
	fmt.Printf("best vram_nz seen: %d\n", bestNZ)
	dumpFB(mem, 0x8C000000, "boot_final.png") // SEE where the boot ended (e.g. menu vs first-boot)
	report(cpu, mmio, start)
}

// runRTBench exercises the Emulator facade + RunRealtime end-to-end: resume from the
// save-state, pace to a target instructions/second for ~3s, and report the achieved rate.
func runRTBench(img []byte) {
	e := NewEmulator(img)
	if raw, err := os.ReadFile(statePath); err == nil {
		if err := e.Resume(raw); err != nil {
			fmt.Println("rtbench: resume failed, running fresh boot:", err)
		} else {
			fmt.Printf("rtbench: resumed from save-state at pc=0x%08x\n", e.PC())
		}
	} else {
		fmt.Println("rtbench: no save-state found — pacing a fresh-boot machine (provision first for the menu)")
	}
	const target, frameHz = 20_000_000, 60
	frames := 0
	stop := make(chan struct{})
	go func() { time.Sleep(3 * time.Second); close(stop) }()
	c0 := e.Cycles()
	t0 := time.Now()
	e.RunRealtime(target, frameHz, func() { frames++ }, stop)
	el := time.Since(t0).Seconds()
	ran := e.Cycles() - c0
	fmt.Printf("rtbench: target=%.0fM ips @ %dHz -> ran %d instr in %.2fs = %.2fM ips, %d frames (%.0f fps), end pc=0x%08x\n",
		float64(target)/1e6, frameHz, ran, el, float64(ran)/el/1e6, frames, float64(frames)/el, e.PC())
}

func report(cpu *CPU, mmio *MMIOBus, start time.Time) {
	// hottest unmapped MMIO — a new unmodeled register (like the battery ADC was)
	// shows up here as a frequently-polled address returning 0.
	if len(mmio.unknown) > 0 {
		type uk struct {
			a uint32
			n int
		}
		var u []uk
		for a, n := range mmio.unknown {
			u = append(u, uk{a, n})
		}
		sort.Slice(u, func(i, j int) bool { return u[i].n > u[j].n })
		fmt.Printf("top unmapped MMIO (addr x count): ")
		for i := 0; i < len(u) && i < 12; i++ {
			fmt.Printf(" 0x%08x:x%d", u[i].a, u[i].n)
		}
		fmt.Println()
	}
	if len(mmio.watchPC) > 0 {
		type pk struct {
			pc uint32
			n  int
		}
		var p []pk
		for pc, n := range mmio.watchPC {
			p = append(p, pk{pc, n})
		}
		sort.Slice(p, func(i, j int) bool { return p[i].n > p[j].n })
		fmt.Printf("readers of 0x%08x region (pc x count): ", mmio.watchBase)
		for i := 0; i < len(p) && i < 10; i++ {
			fmt.Printf(" 0x%08x:x%d", p[i].pc, p[i].n)
		}
		fmt.Println()
	}
	fmt.Printf("model: *0xfd8018d4=0x%08x *0x8c04ca24=0x%08x strap[0xFF000024]=0x%08x (want 0xca02)\n",
		cpu.mem.R32(0xfd8018d4), cpu.mem.R32(0x8c04ca24), cpu.mem.R32(0xFF000024))
	fmt.Printf("is_erased cmp: flash@0x300=0x%08x  *0x806827a4=0x%08x\n",
		cpu.mem.R32(0xa0000300), cpu.mem.R32(0x806827a4))
	el := time.Since(start).Seconds()
	rate := float64(cpu.cycles) / el / 1e6
	fmt.Printf("\n=== ran %d instr in %.2fs (%.1f M instr/s) ===\n", cpu.cycles, el, rate)
	fmt.Printf("PC=0x%08x SR=0x%08x VBR=0x%08x PR=0x%08x irqs=%d ticks=%d fpu_ops=%d\n",
		cpu.pc, cpu.sr, cpu.vbr, cpu.pr, cpu.irqCnt, mmio.timerTicks, cpu.fpuOps)
	// stack dump: scan the current stack for code-pointer return addresses to
	// reconstruct the call chain that parked the OS here.
	// hunt for a rendered framebuffer ANYWHERE in DRAM (distinguishes "drew but
	// never pushed to LCD" from "never drew at all").
	const FB = 396 * 224 * 2
	dram := cpu.mem.dram
	best, bestOff := 0, uint32(0)
	for off := uint32(0); off+FB < DramSize; off += 0x8000 {
		nz := 0
		for k := uint32(0); k < FB; k += 64 {
			if dram[off+k] != 0 || dram[off+k+1] != 0 {
				nz++
			}
		}
		if nz > best {
			best, bestOff = nz, off
		}
	}
	fmt.Printf("densest DRAM framebuffer-window: phys=0x%08x nz=%d/%d (%.0f%%)\n",
		DramBase+bestOff, best, FB/64, 100*float64(best)/float64(FB/64))
	{ // raw framebuffer bytes for offline stride/orientation exploration
		raw := make([]byte, 0x40000)
		for i := range raw {
			raw[i] = byte(cpu.mem.R8(0x8C000000 + uint32(i)))
		}
		if f, err := os.Create("fb_raw.bin"); err == nil {
			f.Write(raw)
			f.Close()
			fmt.Println("wrote fb_raw.bin (256KB from 0x8c000000)")
		}
	}
	dumpFB(cpu.mem, 0x8C000000, "fb_0c000000.png")
	dumpFB(cpu.mem, 0x8C028800, "fb_0c028800.png")
	dumpFB(cpu.mem, 0x8C088000, "fb_088000.png")
	dumpFB(cpu.mem, 0x8C090000, "fb_090000.png")
	dumpFB(cpu.mem, 0x8C000000|(DramBase+bestOff)&0x1FFFFFFF, "fb_densest.png")

	sp := cpu.r[15]
	fmt.Printf("R15(sp)=0x%08x  call-chain candidates (return addrs on stack):\n", sp)
	for off := uint32(0); off < 0x300; off += 4 {
		w := cpu.mem.R32(sp + off)
		if w >= 0x80001000 && w < 0x80C00000 {
			fmt.Printf("  [sp+0x%03x] 0x%08x\n", off, w)
		}
	}
}

// runFTL hooks the flash translation-layer routines and records their return codes
// (r0 at return) to test whether flash records pass ECC during boot.
func runFTL(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	watch := map[uint32]string{
		0x80371718: "ECC_verify(FUN_80371718)", // ret 1=clean 2=corrected 3=uncorrectable
		0x80370ff0: "block_read(FUN_80370ff0)", // ret 0=ok -5=ECC-fail -2/-6=other
		0x803715dc: "rec_verify(FUN_803715dc)",
		0x801885f2: "load_setup(FUN_801885f2)",   // ret -6 on record-validate fail
		0x8018879c: "validate_rec(FUN_8018879c)", // ret 1=found type0x1d ok, 0=not
	}
	order := []string{"block_read(FUN_80370ff0)", "rec_verify(FUN_803715dc)",
		"ECC_verify(FUN_80371718)", "load_setup(FUN_801885f2)", "validate_rec(FUN_8018879c)"}
	type frame struct {
		name string
		ret  uint32
	}
	var pending []frame
	hist := map[string]map[int32]int{}
	calls := map[string]int{}
	blkHist := map[uint64]int{}
	for _, n := range watch {
		hist[n] = map[int32]int{}
	}
	fmt.Printf("FTL probe: booting up to %d instr, hooking flash ECC/reader returns...\n", maxIns)
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x after %d: %v\n", cpu.pc, cpu.cycles, r)
		}
		fmt.Printf("\n=== FTL return-code histograms (after %d instr) ===\n", cpu.cycles)
		for _, n := range order {
			fmt.Printf("%-28s calls=%d  returns:", n, calls[n])
			for code, c := range hist[n] {
				fmt.Printf("  %d:x%d", code, c)
			}
			fmt.Println()
		}
		// block_read(param_1=r4, param_2=r5): param_2 is the record/block number.
		// distinct count + most-read blocks tells progressing-scan vs stuck-reread.
		fmt.Printf("\nblock_read args: %d distinct (r4,r5) keys\n", len(blkHist))
		type bk struct {
			key uint64
			n   int
		}
		var top []bk
		for k, n := range blkHist {
			top = append(top, bk{k, n})
		}
		sort.Slice(top, func(i, j int) bool { return top[i].n > top[j].n })
		for i := 0; i < len(top) && i < 25; i++ {
			fmt.Printf("  r4=%d r5=0x%x  x%d\n", int32(top[i].key>>32), uint32(top[i].key), top[i].n)
		}
	}()
	for cpu.cycles < maxIns {
		pc := cpu.pc
		if pc == 0x80370ff0 {
			blkHist[uint64(cpu.r[4])<<32|uint64(cpu.r[5])]++
		}
		if name, ok := watch[pc]; ok {
			pending = append(pending, frame{name, cpu.pr})
			calls[name]++
		}
		if len(pending) > 0 && pc == pending[len(pending)-1].ret {
			f := pending[len(pending)-1]
			pending = pending[:len(pending)-1]
			hist[f.name][int32(cpu.r[0])]++
		}
		mmio.tick(cpu)
		cpu.step()
	}
}

// runShellTrace records the control flow through the shell FUN_802aea26 for ONE
// entry (after warmup): every in-function PC that is a branch/call site, and the
// r0 value each time execution returns INTO the shell from a callee. This shows
// which conditional branches the shell takes and which app-launch/draw call is
// skipped. lo..hi bound the shell body; the function returns via rts @0x802af3d8.
func runShellTrace(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	const lo, hi = 0x802aea26, 0x802af3e0
	warm := uint64(0) // capture the FIRST shell entry (the app-launch one, post-ADC-fix)
	armed, done := false, false
	prevIn := false
	var lines []string
	add := func(s string) {
		if len(lines) < 700 {
			lines = append(lines, s)
		}
	}
	fmt.Printf("SHELLTRACE: capturing one FUN_802aea26 entry after warmup=%d\n", warm)
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x: %v\n", cpu.pc, r)
		}
		fmt.Printf("=== shell control-flow trace (%d events) ===\n", len(lines))
		for _, s := range lines {
			fmt.Println(s)
		}
	}()
	for cpu.cycles < maxIns && !done {
		pc := cpu.pc
		in := pc >= lo && pc < hi
		if !armed && in && cpu.cycles >= warm {
			armed = true
			add(fmt.Sprintf("ENTER @%d r4=%d r5=%d", cpu.cycles, cpu.r[4], cpu.r[5]))
		}
		if armed {
			if in && !prevIn {
				// returned into the shell from a callee: log r0
				add(fmt.Sprintf("  <- ret pc=0x%08x r0=0x%x (%d)", pc, cpu.r[0], int32(cpu.r[0])))
			}
			if in {
				op := mem.R16(pc)
				// log branch/call ops to keep the trace readable
				hb := op >> 12
				if hb == 0x8 /*bt/bf*/ || hb == 0xA /*bra*/ || hb == 0xB /*bsr*/ ||
					op == 0x000b /*rts*/ || (op&0xF0FF) == 0x402b /*jmp @Rn*/ ||
					(op&0xF0FF) == 0x400b /*jsr @Rn*/ {
					add(fmt.Sprintf("    0x%08x op=%04x", pc, op))
				}
				if pc == 0x802af3d8 { // rts
					add(fmt.Sprintf("RETURN @%d", cpu.cycles))
					done = true
				}
			}
			prevIn = in
		}
		mmio.tick(cpu)
		cpu.step()
	}
}

// runDraw watches the framebuffer (DRAM @ the pushed SAR) for ANY change and logs
// the PC + nonzero-count when it changes — finding whether/where the OS draws after
// the initial screen-clear. Checks every 2048 instr.
func runDraw(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	const fbCached = 0x8C000000 // the SAR pushed after the battery-ADC fix (full buffer base)
	const fbLen = 396 * 224 * 2
	snap := func() (uint64, int) {
		var h uint64 = 1469598103934665603
		nz := 0
		for k := uint32(0); k < fbLen; k += 2 {
			v := mem.R16(fbCached + k)
			if v != 0 {
				nz++
			}
			h = (h ^ uint64(v)) * 1099511628211
		}
		return h, nz
	}
	last, _ := snap()
	changes := 0
	fmt.Printf("DRAW: watching FB @0x%08x (%d B) for changes over %d instr\n", fbCached, fbLen, maxIns)
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x after %d: %v\n", cpu.pc, cpu.cycles, r)
		}
		fmt.Printf("=== %d framebuffer changes detected ===\n", changes)
	}()
	for cpu.cycles < maxIns {
		mmio.tick(cpu)
		cpu.step()
		if cpu.cycles&0x7FF == 0 {
			h, nz := snap()
			if h != last {
				changes++
				if changes <= 60 {
					fmt.Printf("  [%11d] FB changed: nz=%d pc=0x%08x pr=0x%08x\n", cpu.cycles, nz, cpu.pc, cpu.pr)
				}
				last = h
			}
		}
	}
}

// runGate counts entries to specific functions in the 3.60 os_main_loop and
// histograms their return value (r0 at the matching return PC), to see which
// gate condition prevents the menu-drawing shell (FUN_802aea26) from running.
func runGate(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	names := map[uint32]string{
		0x8035e1be: "FUN_8035e1be",
		0x80056236: "s_80056236(7)",
		0x8035d234: "s_8035d234",
		0x8035cf58: "s_8035cf58",
		0x8035c7c0: "s_8035c7c0",
		0x8035cbfc: "s_8035cbfc",
		0x801de9ca: "is_erased",
	}
	order := []uint32{0x8035e1be, 0x80056236, 0x8035d234, 0x8035cf58, 0x8035c7c0, 0x8035cbfc, 0x801de9ca}
	calls := map[uint32]int{}
	rets := map[uint32]map[int32]int{}
	for a := range names {
		rets[a] = map[int32]int{}
	}
	type fr struct {
		a   uint32
		ret uint32
	}
	var stk []fr
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x after %d: %v\n", cpu.pc, cpu.cycles, r)
		}
		fmt.Printf("\n=== os_main_loop gate counts (after %d instr) ===\n", cpu.cycles)
		for _, a := range order {
			fmt.Printf("%-32s calls=%-8d returns:", names[a], calls[a])
			for code, c := range rets[a] {
				fmt.Printf("  %d:x%d", code, c)
			}
			fmt.Println()
		}
	}()
	for cpu.cycles < maxIns {
		pc := cpu.pc
		if nm, ok := names[pc]; ok && nm != "" {
			calls[pc]++
			stk = append(stk, fr{pc, cpu.pr})
		}
		if n := len(stk); n > 0 && pc == stk[n-1].ret {
			rets[stk[n-1].a][int32(cpu.r[0])]++
			stk = stk[:n-1]
		}
		mmio.tick(cpu)
		cpu.step()
	}
}

// runDrive boots to the UI, injects a KEYSC keypress (configured on mmio), and dumps
// a PNG every 15M instr so we can WATCH the screen respond. fb @0x8c000000, 384x216.
func runDrive(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	fmt.Printf("DRIVE: kbReg=%d kbVal=0x%x press@[%d,%d) maxIns=%d\n",
		mmio.kbReg, mmio.kbVal, mmio.kbStart, mmio.kbEnd, maxIns)
	next := uint64(15_000_000)
	frame := 0
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x: %v\n", cpu.pc, r)
		}
		dumpFB(mem, 0x8C000000, fmt.Sprintf("drive_final.png"))
	}()
	for cpu.cycles < maxIns {
		mmio.tick(cpu)
		cpu.step()
		if cpu.cycles >= next {
			dumpFB(mem, 0x8C000000, fmt.Sprintf("drive_%02d.png", frame))
			frame++
			next += 15_000_000
		}
	}
}

// Key-event queue layout (3.60 keyboard driver @0x801e0000). Each DAT below holds
// a runtime pointer/base set up at driver init; the enqueue FUN_801e684c and the
// peek FUN_801e6994 operate on these. We read them to inspect/inject the queue.
const (
	kqCountPtr = 0x801e6a1c // -> int* count (entries currently queued, 0..16)
	kqWidxPtr  = 0x801e6a20 // -> int* write index (0..15, wraps)
	kqRowBase  = 0x801e6a24 // -> byte[16] row buffer base (stores row+1)
	kqColBase  = 0x801e6a28 // -> byte[16] col buffer base (stores col+1)
	kqModBase  = 0x801e6a2c // -> byte[16] modifier buffer base
	kqRidxPtr  = 0x801e6a40 // -> int* read index (consumer side, FUN_801e6994)
)

func dumpKQ(mem *Memory, tag string) {
	cntP := mem.R32(kqCountPtr)
	wiP := mem.R32(kqWidxPtr)
	riP := mem.R32(kqRidxPtr)
	rowB := mem.R32(kqRowBase)
	colB := mem.R32(kqColBase)
	fmt.Printf("  KQ[%s]: cntPtr=0x%08x wiPtr=0x%08x riPtr=0x%08x rowBase=0x%08x colBase=0x%08x\n",
		tag, cntP, wiP, riP, rowB, colB)
	if cntP != 0 && wiP != 0 && riP != 0 && rowB != 0 && colB != 0 {
		fmt.Printf("         count=%d widx=%d ridx=%d  rows=[", mem.R32(cntP), mem.R32(wiP), mem.R32(riP))
		for i := uint32(0); i < 16; i++ {
			fmt.Printf("%02x ", mem.R8(rowB+i))
		}
		fmt.Printf("] cols=[")
		for i := uint32(0); i < 16; i++ {
			fmt.Printf("%02x ", mem.R8(colB+i))
		}
		fmt.Println("]")
	}
}

// runKey boots to the language screen, then injects a single keypress (matrix
// row,col) by CALLING the OS's own scan-enqueue routine FUN_801e684c as a
// subroutine — the faithful path: the key lands in exactly the queue the UI
// consumes, with the driver's own dedup/format. Dumps a PNG every 15M instr.
func runKey(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64, row, col uint32, pressAt uint64) {
	fmt.Printf("KEY: inject (row=%d,col=%d) at cycle>=%d, maxIns=%d\n", row, col, pressAt, maxIns)
	injected := false
	next := uint64(15_000_000)
	frame := 0
	safe := func() bool {
		// not in an exception (BL clear), not inside the keyboard driver, not in
		// the early vector/dispatch region -> the queue state is consistent.
		if cpu.sr&srBL != 0 {
			return false
		}
		if cpu.pc >= 0x801e0000 && cpu.pc < 0x801f0000 {
			return false
		}
		return cpu.pc >= 0x80002000 && cpu.pc < 0x80c00000
	}
	fbHash := func() uint64 {
		var h uint64 = 1469598103934665603
		for k := uint32(0); k < 384*216*2; k += 2 {
			h = (h ^ uint64(mem.R16(0x8C000000+k))) * 1099511628211
		}
		return h
	}
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x: %v\n", cpu.pc, r)
		}
		dumpKQ(mem, "final")
		fmt.Printf("FBHASH row=%d col=%d -> %016x\n", row, col, fbHash())
		dumpFB(mem, 0x8C000000, "key_final.png")
	}()
	for cpu.cycles < maxIns {
		mmio.tick(cpu)
		cpu.step()
		if !injected && cpu.cycles >= pressAt && safe() {
			fmt.Printf("[%d] injecting at pc=0x%08x sp=0x%08x\n", cpu.cycles, cpu.pc, cpu.r[15])
			dumpKQ(mem, "pre")
			sp := cpu.r[15]
			keybuf := sp - 8
			mem.W8(keybuf, row)
			mem.W8(keybuf+1, col)
			ret := cpu.callInject(0x801e684c, keybuf, 0)
			fmt.Printf("  FUN_801e684c returned %d\n", ret)
			dumpKQ(mem, "post")
			injected = true
		}
		if cpu.cycles >= next {
			dumpKQ(mem, fmt.Sprintf("f%02d", frame))
			dumpFB(mem, 0x8C000000, fmt.Sprintf("key_%02d.png", frame))
			frame++
			next += 15_000_000
		}
	}
}

// injectKey enqueues one matrix keypress (0-based row,col) by calling the OS's
// own scan-enqueue routine FUN_801e684c as a subroutine. Returns its result.
func injectKey(cpu *CPU, mem *Memory, row, col uint32) uint32 {
	sp := cpu.r[15]
	keybuf := sp - 8
	mem.W8(keybuf, row)
	mem.W8(keybuf+1, col)
	return cpu.callInject(0x801e684c, keybuf, 0)
}

// keyQueueCount reads the OS key-event queue length. The count lives at *(*0x801e6a1c)
// (a DRAM pointer set up at boot; see FUN_801e684c). Returns 0 if the pointer isn't a
// plausible DRAM address yet (early boot) so callers treat the queue as drained.
func keyQueueCount(mem *Memory) uint32 {
	p := mem.R32(0x801e6a1c)
	if p < 0x88000000 || p >= 0x8e000000 {
		return 0
	}
	return mem.R32(p)
}

func keySafe(cpu *CPU) bool {
	if cpu.sr&srBL != 0 {
		return false
	}
	if cpu.pc >= 0x801e0000 && cpu.pc < 0x801f0000 {
		return false
	}
	return cpu.pc >= 0x80002000 && cpu.pc < 0x80c00000
}

// runScanCap captures how the OS scans the key matrix. With the machine RESUMED at the
// MAIN MENU (idling in getkey), it records every access to the KEYSC/KIU/PFC register
// blocks so we can see the scan protocol (which registers, column-select writes vs
// row-data reads) — the basis for modelling the matrix in mmio.go so injected keys flow
// through the OS's native scan (no flush). Writes summary + full trace to re/scancap.txt.
func runScanCap(cpu *CPU, mem *Memory, mmio *MMIOBus) {
	step := func(n int) {
		for i := 0; i < n; i++ {
			mmio.tick(cpu)
			cpu.step()
		}
	}
	fmt.Printf("scancap: warmup (resume settle) from pc=0x%08x ...\n", cpu.pc)
	step(1_000_000) // let the resumed machine reach its steady idle getkey loop
	mmio.scanCount = map[string]int{}
	mmio.scanSeq = nil
	mmio.scanCap = true
	step(3_000_000) // capture window
	mmio.scanCap = false
	fmt.Printf("scancap: captured %d accesses (%d unique reg/dir)\n", len(mmio.scanSeq), len(mmio.scanCount))

	// summary histogram, sorted by count desc
	type kv struct {
		k string
		v int
	}
	var hist []kv
	for k, v := range mmio.scanCount {
		hist = append(hist, kv{k, v})
	}
	sort.Slice(hist, func(i, j int) bool { return hist[i].v > hist[j].v })
	var sb strings.Builder
	sb.WriteString("=== KEYSC/KIU/PFC access histogram (count desc) ===\n")
	for _, e := range hist {
		sb.WriteString(fmt.Sprintf("  %-14s x%d\n", e.k, e.v))
	}
	sb.WriteString("\n=== ordered access trace (cyc: PC R/W REGION+off [size] = val) ===\n")
	for _, e := range mmio.scanSeq {
		rw := "R"
		if e.write {
			rw = "W"
		}
		sb.WriteString(fmt.Sprintf("%9d: %08x %s %s+%03x [%d] = %08x\n", e.cyc, e.pc, rw, e.region, e.off, e.size, e.val))
	}
	out := "../re/scancap.txt"
	if err := os.WriteFile(out, []byte(sb.String()), 0644); err != nil {
		fmt.Println("scancap: write error:", err)
	} else {
		fmt.Printf("scancap: wrote %s\n", out)
	}
	// also echo the histogram to stdout for an immediate read
	fmt.Print(sb.String()[:min(len(sb.String()), 1600)])
}

// runSeq drives the first-boot setup UI by injecting a SEQUENCE of keypresses
// ("row-col,row-col,...", 0-based matrix coords) spaced `interval` instructions
// apart starting at `pressAt`, dumping a PNG after each so we can watch the setup
// flow advance toward the main menu. Known keys (grid C,R -> inject row=R-1,col=C-1):
//
//	DOWN = C8R3 (2-7) | advance/Next = C10R2 (1-9) | SHIFT = C9R7 (6-8)
func runSeq(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64, seqStr string, pressAt, interval uint64, watch bool) {
	type key struct {
		row, col uint32
		gap      uint64 // instrs to wait AFTER this key before the next (0 => default interval)
		wait     bool   // if set: after pressing, wait until the OS key queue drains, then wait `gap` (settle)
	}
	var seq []key
	for _, tok := range splitComma(seqStr) {
		// token forms: "r-c" (default interval), "r-c*GAP" (fixed gap before NEXT key),
		// or "r-c*wGAP" (consume-gated: wait until the key is read out of the OS queue,
		// then settle GAP instrs before the next key — robust against redraw absorption).
		var r, c uint32
		var gap uint64
		wait := false
		if star := indexByte(tok, '*'); star >= 0 {
			rest := tok[star+1:]
			if len(rest) > 0 && (rest[0] == 'w' || rest[0] == 'W') {
				wait = true
				rest = rest[1:]
			}
			gap = parseUint(rest)
			tok = tok[:star]
		}
		if n, _ := fmt.Sscanf(tok, "%d-%d", &r, &c); n == 2 {
			seq = append(seq, key{r, c, gap, wait})
		}
	}
	fmt.Printf("SEQ: %d keys %v, press@%d every %d, maxIns=%d\n", len(seq), seq, pressAt, interval, maxIns)
	// BCD operand read-watch: the literal "98765" is tokenised to BCD bytes 49 87 65
	// (exp nibble 4 + mantissa). We scan DRAM for that signature, lock a watch window
	// onto it, and histogram which PCs READ it during eval (EXE). bcdPattern must match
	// the literal actually typed in seqStr.
	bcdPattern := []byte{0x49, 0x87, 0x65} // 98765 result -> BCD ..49 87 65 (distinctive; in Ans region)
	bcdLocked := false
	nextScan := pressAt
	bcdBaseline := map[uint32]bool(nil) // pre-existing hits (captured before typing) to exclude
	scanBCD := func() []uint32 {
		var hits []uint32
		d := mem.dram
		for i := 0; i+len(bcdPattern) <= len(d); i++ {
			if d[i] == bcdPattern[0] && d[i+1] == bcdPattern[1] && d[i+2] == bcdPattern[2] {
				hits = append(hits, DramBase+uint32(i))
			}
		}
		return hits
	}
	idx := 0
	nextPress := pressAt
	frame := 0
	nextFrame := pressAt + interval/2
	draining := false // waiting for the just-pressed key to actually be DECODED by the app
	drainGap := uint64(0)
	drainStart := uint64(0)
	var lastRow, lastCol uint32
	sawDecode := false
	retries := 0
	captured := false // formatter-snapshot taken (oracle-diff capture, harness-only)
	// eval call-tree trace: armed when the LAST seq key (EXE) is injected; logs jsr/bsr
	// targets (+args) and rts return values for ~80M instr, excluding the heavy redraw
	// regions, so we can see which BCD routine returns 0 where it should return the literal.
	var evalStart uint64
	var fpuAtEval uint64
	var callLog *os.File
	callLines := 0
	depth := 0
	defer func() {
		if callLog != nil {
			callLog.Close()
			fmt.Printf("=== eval call-trace: %d lines -> eval_calls.txt ===\n", callLines)
		}
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x: %v\n", cpu.pc, r)
		}
		var h uint64 = 1469598103934665603
		for k := uint32(0); k < 384*216*2; k += 2 {
			h = (h ^ uint64(mem.R16(0x8C000000+k))) * 1099511628211
		}
		fmt.Printf("FBHASH seq -> %016x\n", h)
		fmt.Printf("fpu_ops total=%d  (eval+format window: evalStart fpu=%d)\n", cpu.fpuOps, fpuAtEval)
		dumpFB(mem, 0x8C000000, "seq_final.png")
		if watch && mem.rdPC != nil {
			type pk struct {
				pc uint32
				n  int
			}
			var ps []pk
			for pc, n := range mem.rdPC {
				ps = append(ps, pk{pc, n})
			}
			sort.Slice(ps, func(i, j int) bool { return ps[i].n > ps[j].n })
			fmt.Printf("=== BCD operand-read PCs (window 0x%08x-0x%08x), top 30 ===\n", mem.rdLo, mem.rdHi)
			for i := 0; i < len(ps) && i < 30; i++ {
				fmt.Printf("  pc=0x%08x (instr~0x%08x)  x%d\n", ps[i].pc, ps[i].pc-2, ps[i].n)
			}
		}
		if watch { // scan ALL DRAM for BCD operands/results to settle formatter-vs-eval
			scan3 := func(label string, p0, p1, p2 byte) {
				d := mem.dram
				var hits []uint32
				for i := 0; i+2 < len(d); i++ {
					if d[i] == p0 && d[i+1] == p1 && d[i+2] == p2 {
						hits = append(hits, DramBase+uint32(i))
					}
				}
				fmt.Printf("  BCD %s (%02x %02x %02x): %d hit(s)", label, p0, p1, p2, len(hits))
				for i := 0; i < len(hits) && i < 6; i++ {
					fmt.Printf(" 0x%08x", hits[i])
				}
				fmt.Println()
			}
			fmt.Printf("=== end-of-run DRAM BCD scan (operands/result of 2+3) ===\n")
			scan3("2 ", 0x10, 0x02, 0x00)
			scan3("3 ", 0x10, 0x03, 0x00)
			scan3("5 ", 0x10, 0x05, 0x00)
			scan3("98765", 0x10, 0x49, 0x87)
		}
		if watch { // hexdump the Ans/history region so we can read the STORED result BCD directly
			fmt.Printf("=== Ans/history region 0x0c0d7e00-0x0c0d8200 (hex + ascii) ===\n")
			for base := uint32(0x0c0d7e00); base < 0x0c0d8200; base += 16 {
				row := ""
				asc := ""
				for i := uint32(0); i < 16; i++ {
					b := mem.R8(base + i)
					row += fmt.Sprintf("%02x ", b)
					if b >= 0x20 && b < 0x7f {
						asc += string(rune(b))
					} else {
						asc += "."
					}
				}
				fmt.Printf("  0x%08x: %s |%s|\n", base, row, asc)
			}
		}
	}()
	for cpu.cycles < maxIns {
		mmio.tick(cpu)
		if watch && evalStart > 0 && callLog != nil && cpu.cycles >= evalStart &&
			cpu.cycles < evalStart+200_000_000 && callLines < 150000 {
			pc := cpu.pc
			// WHITELIST: BCD/number library + the calc/format module (0x800e..0x80100000) + the
			// render/display region (0x80050..0x80060000), so we can catch the result FORMATTER.
			inNumLib := (pc >= 0x801da000 && pc < 0x801dc000) || (pc >= 0x80200000 && pc < 0x80212000) ||
				(pc >= 0x800e0000 && pc < 0x80100000) || (pc >= 0x80050000 && pc < 0x80060000)
			if inNumLib {
				op := mem.R16(pc)
				// dump8 shows the 8-byte BCD struct at a DRAM pointer arg (the number's value).
				dump8 := func(a uint32) string {
					if a&0x1FFFFFFF >= DramBase && a&0x1FFFFFFF < DramBase+DramSize {
						return fmt.Sprintf("%02x%02x%02x%02x%02x%02x%02x%02x",
							mem.R8(a), mem.R8(a+1), mem.R8(a+2), mem.R8(a+3),
							mem.R8(a+4), mem.R8(a+5), mem.R8(a+6), mem.R8(a+7))
					}
					return "-"
				}
				logCall := func(tgt uint32) {
					depth++
					fmt.Fprintf(callLog, "[%d] d%-2d CALL 0x%08x -> 0x%08x  r4=0x%x[%s] r5=0x%x[%s] r6=0x%x r7=0x%x\n",
						cpu.cycles, depth, pc, tgt, cpu.r[4], dump8(cpu.r[4]), cpu.r[5], dump8(cpu.r[5]), cpu.r[6], cpu.r[7])
					callLines++
				}
				switch {
				case op&0xF000 == 0xB000: // bsr disp
					disp := int32(op & 0xFFF)
					if disp&0x800 != 0 {
						disp |= ^int32(0xFFF)
					}
					logCall(uint32(int32(pc) + 4 + disp*2))
				case op&0xF0FF == 0x400B: // jsr @Rn
					logCall(cpu.r[(op>>8)&0xF])
				case op&0xF0FF == 0x0003: // bsrf Rn
					logCall(pc + 4 + cpu.r[(op>>8)&0xF])
				case op == 0x000b: // rts — show r0 AND the 8-byte struct r4/r5 still point at
					fmt.Fprintf(callLog, "[%d] d%-2d RET  0x%08x  r0=0x%x (%d)  r4=0x%x[%s] r5=0x%x[%s]\n",
						cpu.cycles, depth, pc, cpu.r[0], int32(cpu.r[0]), cpu.r[4], dump8(cpu.r[4]), cpu.r[5], dump8(cpu.r[5]))
					callLines++
					depth--
				}
			}
		}
		// Formatter-entry capture for the Python oracle diff (cont.17c): the result
		// FORMATTER is FUN_800fc5a4, called with r4 -> the result BCD (98765 -> 10 49 87 65).
		// On the first armed entry with that exact BCD, snapshot + lockstep-trace, then stop.
		if watch && evalStart > 0 && !captured && cpu.pc == 0x800fc5a4 && mem.R32(cpu.r[4]) == 0x10498765 {
			captured = true
			captureFormatter(cpu, mem, 2_000_000)
			break
		}
		cpu.step()
		if watch && !bcdLocked && cpu.cycles >= nextScan {
			nextScan = cpu.cycles + 300_000
			all := scanBCD()
			if bcdBaseline == nil { // first scan: record pre-existing hits, don't lock on them
				bcdBaseline = map[uint32]bool{}
				for _, h := range all {
					bcdBaseline[h] = true
				}
				fmt.Printf("[%d] BCD baseline: %d pre-existing hit(s) excluded\n", cpu.cycles, len(all))
				all = nil
			}
			var hits []uint32 // only freshly-appeared (typed) hits, PREFERRING the Ans/result
			// region 0x0c0d8000-0x0c0d9000 (where string→BCD writes the evaluated number),
			// not the editor token copies at 0x0c186xxx — so we watch the RESULT during display.
			for _, h := range all {
				if !bcdBaseline[h] && h >= 0x0c0d8000 && h < 0x0c0d9000 {
					hits = append(hits, h)
				}
			}
			if len(hits) > 0 {
				// TIGHT window on just the first literal copy (16-byte aligned, 0x20 span),
				// so we don't overlap the MathIO display struct at ~0x0c1862xx which the
				// redraw reads millions of times and swamps the operand-fetch signal.
				lo := hits[0] &^ 0xFF
				hi := lo + 0x100
				mem.rdLo, mem.rdHi = lo, hi
				mem.rdPC = map[uint32]int{}
				mem.rdLog = 600
				bcdLocked = true
				fmt.Printf("[%d] BCD literal located: %d hit(s) %v -> watch [0x%08x,0x%08x)\n",
					cpu.cycles, len(hits), hits, lo, hi)
			}
		}
		// decode-confirmed pacing: a key only "lands" if the app's getkey actually reads it
		// (the OS decode FUN_801952cc @0x801952cc runs). A key injected while the app is
		// redrawing is flushed and never decoded -> we re-inject it. This defeats the phase-
		// locking that fixed delays suffer from.
		if draining {
			if cpu.pc >= 0x801952cc && cpu.pc < 0x801952e0 {
				sawDecode = true
			}
			if sawDecode {
				draining = false
				nextPress = cpu.cycles + drainGap
			} else if cpu.cycles-drainStart > 40_000_000 && keySafe(cpu) {
				retries++
				if retries > 8 {
					draining = false
					nextPress = cpu.cycles + drainGap
				} else {
					injectKey(cpu, mem, lastRow, lastCol)
					drainStart = cpu.cycles
					fmt.Printf("[%d]   re-inject (row=%d,col=%d) retry#%d\n", cpu.cycles, lastRow, lastCol, retries)
				}
			}
		}
		if !draining && idx < len(seq) && cpu.cycles >= nextPress && keySafe(cpu) {
			k := seq[idx]
			ret := injectKey(cpu, mem, k.row, k.col)
			fmt.Printf("[%d] key#%d (row=%d,col=%d) -> %d (pc=0x%08x, qc=%d)\n", cpu.cycles, idx, k.row, k.col, ret, cpu.pc, keyQueueCount(mem))
			idx++
			if watch && idx == len(seq) && evalStart == 0 { // last key = EXE -> arm eval trace
				evalStart = cpu.cycles
				fpuAtEval = cpu.fpuOps
				callLog, _ = os.Create("eval_calls.txt")
				fmt.Printf("[%d] eval call-trace armed (final EXE injected)\n", cpu.cycles)
			}
			if k.wait {
				draining = true
				sawDecode = false
				retries = 0
				lastRow, lastCol = k.row, k.col
				drainGap = k.gap
				if drainGap == 0 {
					drainGap = interval
				}
				drainStart = cpu.cycles
				nextPress = cpu.cycles + 0xffffffffffff // parked until drain completes
			} else if k.gap != 0 {
				nextPress += k.gap
			} else {
				nextPress += interval
			}
		}
		if cpu.cycles >= nextFrame {
			dumpFB(mem, 0x8C000000, fmt.Sprintf("seq_%02d.png", frame))
			frame++
			nextFrame += interval
		}
	}
}

func indexByte(s string, b byte) int {
	for i := 0; i < len(s); i++ {
		if s[i] == b {
			return i
		}
	}
	return -1
}

func splitComma(s string) []string {
	var out []string
	cur := ""
	for _, ch := range s {
		if ch == ',' {
			out = append(out, cur)
			cur = ""
		} else {
			cur += string(ch)
		}
	}
	if cur != "" {
		out = append(out, cur)
	}
	return out
}

// runFlashWr histograms FLASH write targets (which we currently ignore). If the FS
// mount/format is trying to program/erase flash, this reveals it — confirming whether
// "ignore flash writes" is what breaks the fls0 mount. Logs first writes in detail.
func runFlashWr(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	warm := maxIns / 4
	mem.fwrites = map[uint32]int{}
	mem.fwLog = 40
	fmt.Printf("FLASHWR: %d instr, logging flash writes (warmup=%d for histogram)\n", maxIns, warm)
	armed := false
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x: %v\n", cpu.pc, r)
		}
		type kv struct {
			page uint32
			n    int
		}
		var top []kv
		var total int
		for p, n := range mem.fwrites {
			top = append(top, kv{p, n})
			total += n
		}
		sort.Slice(top, func(i, j int) bool { return top[i].n > top[j].n })
		fmt.Printf("=== flash write pages (32KB), total=%d ===\n", total)
		for i := 0; i < len(top) && i < 20; i++ {
			fmt.Printf("  phys=0x%08x  x%d\n", top[i].page, top[i].n)
		}
	}()
	for cpu.cycles < maxIns {
		if !armed && cpu.cycles >= warm {
			mem.fwrites = map[uint32]int{} // reset to capture steady-state only
			armed = true
		}
		mmio.tick(cpu)
		cpu.step()
	}
}

// runWmap histograms DRAM write targets (by 32KB page) after a warmup, to find
// WHERE the menu actually draws (the visible buffers @0x8c000000/0x8c028800 stay
// black, so the menu paints elsewhere). Top pages reveal the real draw buffer.
func runWmap(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	warm := maxIns / 4
	fmt.Printf("WMAP: %d instr, histogram DRAM write pages after warmup=%d\n", maxIns, warm)
	armed := false
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x: %v\n", cpu.pc, r)
		}
		type kv struct {
			page uint32
			n    int
		}
		var top []kv
		for p, n := range mem.wpages {
			top = append(top, kv{p, n})
		}
		sort.Slice(top, func(i, j int) bool { return top[i].n > top[j].n })
		fmt.Printf("=== hottest DRAM write pages (32KB) ===\n")
		for i := 0; i < len(top) && i < 20; i++ {
			fmt.Printf("  cached=0x%08x  x%d\n", 0x8C000000+top[i].page, top[i].n)
		}
	}()
	for cpu.cycles < maxIns {
		if !armed && cpu.cycles >= warm {
			mem.wpages = map[uint32]int{}
			armed = true
		}
		mmio.tick(cpu)
		cpu.step()
	}
}

// runStack periodically scans the stack for code return-addresses and histograms
// them. Frequent high-level frames reveal the stable steady-state call chain (the
// loop anchor), which the PC histogram can't show. Skips the first 1/4 (boot).
func runStack(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	hist := map[uint32]uint64{}
	var samples uint64
	warm := maxIns / 4
	fmt.Printf("STACK: %d instr, sampling return-addrs after warmup=%d\n", maxIns, warm)
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x after %d: %v\n", cpu.pc, cpu.cycles, r)
		}
		type kv struct {
			pc uint32
			n  uint64
		}
		var top []kv
		for pc, n := range hist {
			top = append(top, kv{pc, n})
		}
		sort.Slice(top, func(i, j int) bool { return top[i].n > top[j].n })
		fmt.Printf("\n=== return-addrs seen on stack (%d samples) — pct = in how many stacks ===\n", samples)
		for i := 0; i < len(top) && i < 45; i++ {
			fmt.Printf("  0x%08x  %5.1f%%  (x%d)\n", top[i].pc, 100*float64(top[i].n)/float64(samples), top[i].n)
		}
	}()
	for cpu.cycles < maxIns {
		mmio.tick(cpu)
		cpu.step()
		if cpu.cycles >= warm && cpu.cycles&0x3FFF == 0 { // sample every ~16k instr
			samples++
			sp := cpu.r[15]
			seen := map[uint32]bool{}
			for off := uint32(0); off < 0x400; off += 4 {
				w := mem.R32(sp + off)
				// code addr in OS (0x80001000..0x80c00000), dedup per-stack
				if w >= 0x80001000 && w < 0x80C00000 && !seen[w] {
					seen[w] = true
					hist[w]++
				}
			}
		}
	}
}

// runProf samples PC into 64-byte buckets to find the steady-state hot loop.
// Skips the first 1/4 (boot) so the post-delay churn dominates the histogram.
func runProf(cpu *CPU, mem *Memory, mmio *MMIOBus, maxIns uint64) {
	hist := map[uint32]uint64{}
	warm := maxIns / 4
	fmt.Printf("PROF: %d instr, sampling PC after warmup=%d (64B buckets)\n", maxIns, warm)
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("FAULT @0x%08x after %d: %v\n", cpu.pc, cpu.cycles, r)
		}
		type kv struct {
			pc uint32
			n  uint64
		}
		var top []kv
		var total uint64
		for pc, n := range hist {
			top = append(top, kv{pc, n})
			total += n
		}
		sort.Slice(top, func(i, j int) bool { return top[i].n > top[j].n })
		fmt.Printf("\n=== hottest PC buckets (sampled %d) ===\n", total)
		for i := 0; i < len(top) && i < 40; i++ {
			fmt.Printf("  0x%08x  %8d  %5.1f%%\n", top[i].pc, top[i].n, 100*float64(top[i].n)/float64(total))
		}
	}()
	for cpu.cycles < maxIns {
		mmio.tick(cpu)
		cpu.step()
		if cpu.cycles >= warm {
			hist[cpu.pc&^0x3F]++
		}
	}
}

func parseUint(s string) uint64 {
	base := 10
	if len(s) > 2 && (s[0:2] == "0x" || s[0:2] == "0X") {
		s = s[2:]
		base = 16
	}
	v, err := strconv.ParseUint(s, base, 64)
	if err != nil {
		return 0
	}
	return v
}
