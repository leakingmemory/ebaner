# Scripting

The overlay's files hold facts: a crossing, a signal, a TXP position. Rules, such as a
timetable or when a station is manned, are meant to be written as a Lua script:
`<dataset>/overlay/overlay.lua`. It needs ebaner built with Lua 5.4.

```lua
print("God dag frå Nordlandsbanen!")
```

The simulator runs the script once at startup, after the world, the track, the switches
and the rest of the overlay are loaded.

| | Console |
|---|---|
| no script | nothing |
| it ran | `[Script] ran overlay.lua` |
| it failed | `[Script] overlay.lua: <the Lua message>`, and the simulator carries on |
| a script, but built without Lua | `script: overlay.lua present but built without Lua; not run` |

- The interpreter stays open after the script returns, so what it defines stays defined.
- The full standard library is loaded, `io` and `os` included. The script is part of the
  user's own dataset, so it isn't sandboxed.
- **Nothing of the simulator is exposed yet**: no trains, signals, stations or TXP network,
  and no `init` or `tick` hooks.
- The track editor doesn't run scripts.
