## Project Status

- `3dmodels/`: Iteration 1 complete; iteration 2 WIP
  - Considering different potentiometer & button models (pot.s too small for knobs)
  - Looking into overhead enclosure for button/knob interface (i.e., transitioning from partially exposed to fully enclosed)
- `firmware/`: Iteration 1 WIP
  - Currently troubleshooting issues with SPI comm.; assumed resolution w/ the inclusion of tri-state buffer
  - Potentiometers laggy; assumed resolution w/ swap for rotary encoders
- `hardware/`: Iteration 1 complete; iteration 2 WIP
  - Adjustments needed to account for pullups/pulldowns & tri-state buffer
  - Considering swapping potentiometers for rotary encoders & using different buttons
  - Potentially re-evaluate netclasses/copper trace thicknesses
  - Flip around opAmp (for some reason was inverted in schematics)
  - Ensure all passives' values *actually exist* (if not, modify circuit to use ones that do)
  - Swap 5V LDO to buck converter (& generally re-evaluate power distribution method (currently not adequate for long-term usage))

## Project Leads

- `3dmodels/`: [@kaylauyema](https://github.com/kaylauyema)
  - *See [Onshape](https://cad.onshape.com/documents/8ccdb21e8d1079c729b3d0f6/w/fb9eb84c8df37a115b39a8e3/e/654db45d68c6c467c0a4227c?renderMode=0&uiState=67cde4469e5450083d885918) for all CAD files*
- `firmware/`: [@StPelican42](https://github.com/StPelican42)
- `hardware/`: [@Niftion](https://github.com/Niftion)
