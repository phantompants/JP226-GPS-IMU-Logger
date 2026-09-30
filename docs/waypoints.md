# Waypoints, POIs and M5Dial

The Cardputer owns the GPS fix, clock, SD card and waypoint history. It works
with either local GPS or the AtomS3 remote source. The Dial never invents its
own location; it requests that the Cardputer capture its current normalized
telemetry snapshot.

## Cardputer controls

Press `P` for the WAYPOINT page. `A` saves a generic waypoint immediately;
the screen then shows its assigned ID. `N` edits its name, `T` its note and
`F` its photo filename/reference. Type on the Cardputer keyboard, press
Enter to save or Esc to cancel. `V` toggles the last-waypoint details.
Fields are limited to 64 printable ASCII characters; CSV-sensitive commas
and quotation marks are quoted safely. GPS fix, valid time and writable SD
are required for waypoint creation. Editing does not change the original
position, timestamp or ID.

## Dial firmware

Build and flash the separate target:

```powershell
platformio run -d m5dial_waypoint
platformio run -d m5dial_waypoint -t upload --upload-port COMx
```

The Dial scans 2.4 GHz channels for the Cardputer discovery beacon, then
displays link state and receives the Cardputer's status. On DRIVE, a short
press opens WAYPOINT; a long press cycles DRIVE -> WAYPOINT -> SETTINGS.
On WAYPOINT, rotate to select Generic, Photo, Camp, Fuel, Lookout, Track,
Hazard, Interesting or Test Point; press once to capture immediately. The
Dial shows the Cardputer-assigned ID after acknowledgement. An unconfirmed
request is retried with the same sequence; the Cardputer does not create
duplicate waypoints from retries. If the Dial says `NO CONFIRMATION`, check
the Cardputer's last waypoint before pressing again.

On SETTINGS, rotate to select road type, front/rear tyre set pressures,
front/rear suspension settings or vehicle load. Press to edit, rotate to
change, press to send. These values persist on the Cardputer and are
snapshotted into subsequent waypoint rows. DRIVE shows speed, fix,
satellites, logging mode, elapsed time since the first valid fix this boot,
road/tyre/suspension/load values and a short POI. Elapsed time resets after
a Cardputer reboot. The Dial is an optional remote: Cardputer logging and
keyboard waypoints work without it.

ESP-NOW v1 packets are unencrypted and unauthenticated. For a known Dial,
set `config::kDialEspNowMac` in `include/Config.h` to its station MAC before
flashing the Cardputer. The all-zero default learns the first Dial peer until
reboot; do not rely on it in a hostile radio environment.

## SD data

`/telemetry/waypoints_YYYY-MM-DD.csv` contains append-only CREATE and EDIT
revisions. Each revision repeats the immutable waypoint ID, original UTC
timestamp and coordinates, plus fix/heading/context, label, note, photo
reference, category, revision number, action and edit timestamp. The date
in the filename follows the configured local timezone. A failed save may
skip an ID; IDs are never intentionally reused.

`/telemetry/events_YYYY-MM-DD.csv` records waypoint creation, renaming,
notes, photo references, context changes and automatic place entry/exit.
`/telemetry/<daily-track>.csv` has `poi`, `poi_source`, `auto_place` and
`waypoint_id`. A user waypoint within 2 km takes precedence over an
automatic place. The automatic name remains separately available in
`auto_place`. There is currently one photo-reference field per waypoint;
the append-only revision scheme can accommodate additional references in
a later schema.

## Optional online place lookup

The default is fully offline. To enable automatic locality names, configure
Wi-Fi and `place_lookup_url` in `/telemetry/logger.cfg` to a
Nominatim-compatible `/reverse` endpoint you are authorized to use. The
lookup runs on a background task and does not gate GPS/IMU or SD logging.
Requests are separated by at least five minutes and, after a recognized
place, at least 2 km of movement. The result is cached; a changed result
creates PLACE_EXIT/PLACE_ENTER events. The returned locality is accepted
only if its reported point is within approximately 2 km of the current GPS
position. This is a **point-proximity approximation**, not a town-boundary
calculation. It may miss large or oddly shaped localities.

The public OpenStreetMap Foundation Nominatim service is deliberately not
preconfigured. If you choose it, comply with its
[usage policy](https://operations.osmfoundation.org/policies/nominatim/),
including identification, caching and request limits. A self-hosted service
or an offline SD locality database can later implement the same
`ILocationProvider` interface. Waypoints recorded offline do not currently
undergo retrospective geocoding.
