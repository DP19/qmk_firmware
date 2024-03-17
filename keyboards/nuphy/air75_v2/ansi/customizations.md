# Customizations & Fixes

This code is forked from [Nuphy's repo](https://github.com/nuphy-src/qmk_firmware) and based from their `nuphy-air75` branch they currently have an open PR for.

A lot of these customizations and fixes were implemented by @jincao1 & @adi4086 and this is mostly an adopted version of their code combined and tweaked for my liking. Please pay them a visit if you like what you see here

## Customizations

- `Fn + B` when held temporarily displays the current battery percentage on the F and number row.
The F row represents the 10s percentage and number row the ones. Example, 35% will light `F1` -> `F3` and `1` -> `5`.
- Side indicators will flash blue for 0.3s when board enters deep sleep.
- Bluetooth connection indicators will be lit blue when establishing connection. This lights the corresponding
BT mode key. No indicator for RF.
- Default side led pattern set to off and zero brightness. The side light pins will not have power in this state. Saving on battery
- 3ms debounce instead of 2ms (potential stability)

### Sleep Mode (inspired by @adi4086 3 mode sleep)

Sleep mode in this version will turn off all leds after 2 minutes then deep sleep after 6 minutes by turning the MCU to low power mode. This saves significant battery over the light sleep currently implemented in Nuphy's branch.

This does mean the first few key presses after the board is in deep sleep will be lost.

During the light sleep phase RF will stay connected and key press events will be captured.

If you use the 2.4 RF receiver, when it's disconnected from your computer the board will go into immediate deep sleep mode. 
* Since I generally use the USB ports on my monitor when I get up and take my laptop this disconnects and puts the board to sleep.

## Fixes

- Removed unused code and eeprom variables to reduce size
- Make calls to default eeprom init functions in place of relying on a flag to be set in the eeconfig
- Fix keyboard randomly crashing/freezing. (@jincao1)
- Fix keyboard not sleeping properly and draining battery. This version sleeps the processor and uses almost no battery on sleep. (@jincao1)
- Fix LED lights not powering down when not used. This increases battery life around 50-70% when LEDs aren't used. Leds are powered off when brightness is 0. (@jincao1)
- Enhance keyboard reports transmission logic to greatly reduce stuck/lost key strokes. It may still occasionally drop/repeat keys but it's rare. (@jincao1)
- Slightly enhanced sidelight refresh intervals for smoother animations. (@jincao1)
- Reduced unused side LED tables to save a chunk of memory. This may be essential to the RF queue as the board only has 16kb memory available - the queue alone uses over 1.2kb. (@jincao1)

## Keymaps

The standard `via` keymap enables all rgb animations and keyboard features. I use the builds of my `DP19` keymap which has the following customizations. You can build on top of this firmware by copying the folder and making your own customizations to those files. See QMK docs for more

- Default RGB set to `Reactive Simple` and Gradient mode is turned on for all `Reactive` effects. This changes the rgb color in a gradient scale with each keypress. 
  - All but two RGB affects are in this keymap, Reactive simple and solid which I use for testing.
- Disabled Keylock, Mousekey, and N key rollover (NKRO). Since I use Mac which doeesn't support NKRO I've added options in the code to not include extra functions since they won't ever be called. This reduces the firmware size
- Added RF_DFU key to `Fn + M + {` binding to easily get RF module into DFU mode for flashing. Must be in USB mode and hold key unti it appears in RF Connect app.