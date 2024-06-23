# Customizations & Fixes

This code is forked from [Nuphy's repo](https://github.com/nuphy-src/qmk_firmware

A lot of these customizations and fixes were implemented by @jincao1 & @ryodeushii and this is mostly an adopted version of their code combined and tweaked for my liking. Please pay them a visit if you like what you see here

## Customizations

- `Fn + B` when held temporarily displays the current battery percentage on the F and number row.
The F row represents the 10s percentage and number row the ones. Example, 35% will light `F1` -> `F3` and `1` -> `5`.
- Side indicators will flash red for 0.3s when board enters deep sleep.
- Bluetooth connection indicators will be lit blue when establishing connection and while showing connected. This lights the corresponding
BT mode key. Green indicator for RF connection
- RGB will be disabled when brightness is 0 for keyboard lights and halo. These are on the same driver so you have to have them both off for this to happen
- Adjustable per key debounce. This value is used in both press and release events
- Full support for via for all custimizations

### Sleep Mode / Debounce

Sleep mode in this version will turn off all leds after 2 minutes then deep sleep after 6 minutes by turning the MCU to low power mode by default. This saves significant battery over the light sleep currently implemented in Nuphy's branch.

This does mean the first few key presses after the board is in deep sleep will be lost.

During the light sleep phase RF will stay connected and key press events will be captured.

If you use the 2.4 RF receiver, when it's disconnected from your computer the board will go into immediate deep sleep mode.

These can be adjusted in via. Turnig deep sleep lower than light sleep will deep sleep the keyboard at the light sleep delay.

Max delay for both is 20 minutes.

## Debounce
This is set to 4ms by default, if you noticed missed and duplicate keypresses. Try adjusting this value up/down.

## Hot Key Adjustment

To make it easier to adjust on the keyboard I've added the same options for light sleep, deep sleep, and debounce.

Each "special" key can be held then combined with the 4 surrounding keys to increase, decrease, show, and reset the corrosponding custimization. See the below table for reference

| Feature   | Light Sleep | Deep Sleep | Debounce |   |
|-----------|-------------|------------|----------|---|
| Hold Keys | `fn + l`    | `fn + s`   | `fn + d` |   |
| Increase  | o           | w          | e        |   |
| decrease  | .           | x          | c        |   |
| show      | ;           | d          | f        |   |
| reset     | k           | a          | s        |   |

When using the "show" buttons the F row will light up the "10s" and the number rows is 1.

IE - if the light sleep is 11 minutes, F1 and 1 will light up.

Reseting will set the value back to the defaults:

Debounce:    `4ms`

Light Sleep: `2m`

Deep Sleep:  `6m`


## Power on animation

The startup animation can be toggled on/off via `fn + p`. Side light will flash red if it's disabled, and green if enabled.

## Fixes

- Removed unused code and eeprom variables to reduce size
- Make calls to default eeprom init functions in place of relying on a flag to be set in the eeconfig
- Various fixes for general stability
- Custom matrix scanning (@ryodeushii)
- custom debounce algo (@ryodeushii)

## Keymaps

The standard `via` keymap enables all rgb animations and keyboard features. I use the builds of my `DP19` keymap which has the following customizations. You can build on top of this firmware by copying the folder and making your own customizations to those files. See QMK docs for more

- Default RGB set to `Reactive Simple` and Gradient mode is turned on for all `Reactive` effects. This changes the rgb color in a gradient scale with each keypress.
  - All but two RGB affects are in this keymap, Reactive simple and solid which I use for testing.
- Side lights are set to SIDE_OFF and brightness 0 to disable them by default
- Disabled Mousekey, and N key rollover (NKRO). Since I use Mac which doeesn't support NKRO I've added options in the code to not include extra functions since they won't ever be called. This reduces the firmware size

If you do not want/need via support you can use the `default` keymap. All adjustment options are avalible via the keyboard directly
