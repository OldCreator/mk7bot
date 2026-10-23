# MK7 Bot

Status: compiles against stand-in headers and the geometry/manager logic is tested on a PC,
but **nothing has run on real hardware**. Expect to fix small API mismatches in your CTRPF version.

## What's here
- `plugin.cpp`: the bot (3GX plugin). Learns the track by trailing the lead CPU kart on lap 1,
  then drives the learned line alone. Self-tunes steering/braking per lap, brakes for sharp corners,
  reverses out when stuck, uses items by situation, draws the planned line on the bottom screen.
- `manager/main.c`: the 3DS app. A = inject, B = eject, X = forget learned tracks.
- `trainer.py`: optional Citra training for a neural net (Citra I/O, env, PPO, export in one file).

## Step 0: fill in the config block at the top of plugin.cpp (required)
Kart list address, struct stride or pointer mode, offsets for x/z/yaw/speed/rank/item, race flag,
course id. Also item IDs in `SetupItems()`. Find them via CTGP-7 source or Citra's debugger.
Mirror the same kart addresses in the ADDR dict in `trainer.py` if you use the training path.

## Build and install (no eShop needed)
1. Get custom firmware: follow 3ds.guide (Luma3DS). Back up NAND when told to.
2. Plugin: devkitPro + CTRPluginFramework template, build `plugin.cpp` to `MK7Bot.3gx`.
   Put it at `SD:/luma/plugins/<MK7 title id>/MK7Bot.3gx.off` (note the `.off`).
   Check your game's title ID folder on a title database; it differs by region.
3. App: devkitPro libctru, build `manager/main.c` using the 3ds-examples application template.
   Run it from the Homebrew Launcher, or make a CIA and install it with FBI to get a HOME menu icon.
4. In the manager press A to inject, then launch Mario Kart 7. Press B to eject.
   Closing the game always removes the bot from memory.

## Using it
Open the plugin menu in game (Select by default), turn on "Bot on/off" in a Vs race with CPUs.
- New track: it trails the lead CPU for lap 1 (status "LEARNING LINE"), saves the line, then drives alone.
- Known track: it drives immediately and keeps tuning ("best cost" on the bottom screen).
- "Forget this track" wipes that course's line and tuning.

## Optional neural net (Citra)
`python trainer.py bin2json line_XXXXXXXX.bin`, then `train`, then `export`; copy `policy.bin` to `SD:/MK7Bot/`
and enable "Neural policy" in the plugin.

## Known limits
- Drifting and mini-turbos are not implemented yet; items use rules, not learning.
- D-pad steering with pulse-width modulation is the default; set HAVE_CIRCLEPAD_INJECT if your CTRPF supports analog.
- Item/aim button mappings (L for item) need checking.
- Online play is gone anyway; this is for offline CPU races only.
