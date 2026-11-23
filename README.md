# Track Tracer

Turn a picture of a race track into Forge blocks for Halo 4 or Halo: Reach, ready to place in [Halo Map Studio](https://github.com/Sopitive/HaloMapStudio).

## What it does

1. **Load your track image.** Use a top-down picture of the track.
2. **Set the scale.** Click across one lane and tell it how wide that is (4 units).
3. **Trace the track.** Click where the track starts, then add straights and turns. Every section carries on from the last, so the track stays smooth.
4. **Add height.** Mark slopes with the Grade tool and set how steep they are.
5. **Add jumps.** Drop jump markers on the track.
6. **Export.** Save a `.hmsprefab` file and paste the two commands (`prefab import`, `prefab place`) into Halo Map Studio's script console. Works with stock HMS.

## Handy features

- **Straight and turn sections.** Type exact lengths and angles, or let a turn bend to wherever you click.
- **5° steps.** Keeps every angle on a multiple of 5° so straights line up with Halo Map Studio's axes.
- **Halo 4 and Halo: Reach.** Halo 4 uses Forge Island pieces; Reach uses Forge World pieces.
- **Path pieces.** Build the track from blocks (4x4, 3x3, 2x2, 2x4, 5x5, 5x1 and more, in full, short and flat heights), bridges, or walls laid flat.
- **Outline pieces.** Line the edges with columns, 5x1 rails (flat or on edge), flat blocks on edge, 1x4 curbs, walls, cover, railings (Reach) or Skyline wall panels (Halo 4).
- **Reference model.** Save the track image as a `.glb` model at the right size and turn, to import under the track in HMS.
- **Start straight.** A starting area that narrows from wide to track width.
- **Layout size.** Shrink or stretch the whole track from 50% to 200%. The track stays the same width.
- **Image rotation.** Straighten a picture that is a few degrees off.
- **Heading readout.** Always shows which way the track is pointing, so you can find your way back to an angle.
- **Autosave.** Your tracing is kept in your browser. Use Save tracing to keep a copy as a file.

## Using it

Open the page, load your image and follow the steps above. The Help panel on the right lists the keyboard shortcuts.

Try it here: https://gdjenks13.github.io/HaloTrackTracer/
