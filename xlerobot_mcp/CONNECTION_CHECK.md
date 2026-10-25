# Connection and controls verification

Verified October 9, 2026. Plan item 1 is complete through the authenticated local MCP client. Native xlerobot tools are not exposed in this chat; use client.py as documented in the installed skill.

- Updated server protocol: 2. Post-restart boot ID: eba179da-9a98-46f9-93a5-8d01d718c00a.
- Code digest: a43bc653be08f8f36c3c06a24d3058c2e794dbc26cf7babc129ef120b37d1682. Server reported no pending code restart.
- Real hardware mode, motion flag ON at operator restart. No motion commands sent for this work. Startup defaults remain motion OFF unless --allow-motion is supplied.
- Fresh left ID1: position1834, 12.1 V, torque0, stationary, no faults.
- Fresh right ID1: position1752, 12.0 V, torque0, stationary, no faults.
- Fresh neck ID7: position2058, goal2059, torque1, 12.1 V, stationary, no faults.
- Fresh neck ID8: position2040, torque0, 12.1 V, stationary, no faults.
- Fresh camera received and visually inspected. Both views upright, but the post-restart room view is very dark; unsuitable for visual motion without improved visibility. Capture duration about13.5 seconds.

Controls completed: one authoritative project configuration; skill delegates to project controllers; temporary 32/91-tick test overrides removed; permanent request-ID ledger with UUID normalization; refusal of motion after code changes until restart; reusable no-retry MCP client; refreshed skill and README.

Tests passed: test_server.py (HTTP discovery, all8 tools, bearer/Origin rejection, nine invalid-input cases, mock command/replay/stop); test_connection_controls.py (six cases covering replay after restart, failed-attempt consumption, concurrent calls/stop during spawn, disabled startup, code change, commissioning gates). Skill validator and no-hardware direct-helper previews passed.

Limits: these are connection/software tests, not physical stop certification. Physical direction, stopping, neck tilt, arm enable/hold, gripper, load capacity and visual manipulation remain later plan items. Do not infer motion authorization from a successful connection or --allow-motion.
