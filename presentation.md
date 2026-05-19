You are a professional slide designer. Create exactly 2 presentation slides for a robotics project called "XPLORE Rover". Output each slide as a separate high-resolution image (16:9 ratio, 1920×1080px).

---

## GLOBAL DESIGN RULES (apply to both slides)

- Background: #0d1117 (very dark navy)
- Primary accent: #00d4ff (electric cyan)
- Secondary accent: #7c3aed (purple) for node boxes
- Warning/highlight: #f59e0b (amber)
- Text: white (#ffffff) for headings, #b0bec5 for body
- Font: Inter or Roboto — headings bold 28–36px, body 18–22px, labels 14–16px
- All boxes: rounded corners (8–12px radius), thin border 1.5px in accent color
- Arrows: 2px, accent color, with arrowheads
- Top-left corner of each slide: small label "XPLORE Rover" in #00d4ff, 14px
- Bottom-right: slide number "1 / 2" or "2 / 2" in gray 14px
- No slide title bar — titles are embedded in the layout

---

## SLIDE 1 — "Architecture & Communication Flow"

**LAYOUT:** 2 columns. Left column = 60% width (flow diagram). Right column = 40% width (mode table + serial note).

### LEFT COLUMN: FLOW DIAGRAM

Draw this exact node graph, top to bottom, with labeled arrows:

**[PC — Opérateur]**
box contains: "rover_gui / controller_node"
color: dark box, cyan border

Two arrows going DOWN-RIGHT into the Pi zone:
- Arrow 1 → labeled `/rover/mode` → points to `mode_manager_node`
- Arrow 2 → labeled `/rover/cmd_vel` → points to `motor_controller_node`

Inside a rounded rectangle labeled **"Raspberry Pi"** (large container box, purple border, semi-transparent fill):

```
┌─ mode_manager_node ─┐   ← amber highlight, labeled "Orchestrator"
│  manages lifecycle   │
│  of all other nodes  │
└──────────────────────┘
    │ activates/deactivates (dashed arrows to the 3 nodes below)
    ▼                    ▼                        ▼
motor_controller_node   arm_node           autonomous_node
(Twist → motor PWM)    (servos + stepper)  (BFS navigation)
```

- `motor_controller_node` → arrow labeled `/rover/motor_cmd` → `serial_bridge_node`
- `arm_node` → arrow labeled `/rover/arm_serial_cmd` → `serial_bridge_node`
- `autonomous_node` → arrow labeled `/rover/cmd_vel` going LEFT back to `motor_controller_node`
  *(this arrow is CYAN and DASHED — only active in autonomous mode)*

```
┌─ serial_bridge_node ──────────────────────────────────┐
│  "Sole owner of the serial port"                      │
│  highlighted with amber border                        │
└───────────────────────────────────────────────────────┘
```

Bidirectional arrow going DOWN from `serial_bridge_node`:
- ↓ labeled `UART USB — 18 bytes` (9 × int16 : 4 servos, 4 motors, 1 stepper)
- ↑ labeled `30 bytes back` (IMU ×6, encoders ×4, ultrasonic ×5)

**[Arduino ESP32]**
box at bottom, dark fill, cyan border
sub-labels: "Motors · Servos · Stepper"

---

### RIGHT COLUMN

**TOP — Mode table** (title: "Modes" in bold cyan):

| Mode       | motor | arm | aruco | autonomous |
|------------|:-----:|:---:|:-----:|:----------:|
| idle       |  ✗   |  ✗  |  ✗   |     ✗      |
| race       |  ✓   |  ✗  |  ✗   |     ✗      |
| arm        |  ✓   |  ✓  |  ✗   |     ✗      |
| autonomous |  ✓   |  ✓  |  ✓   |     ✓      |

✓ = cyan · ✗ = gray · active cells have subtle cyan background tint

**BOTTOM — Callout box** (amber border):
> "serial_bridge_node is the only node that touches the serial port. All others communicate exclusively via ROS2 topics."

---

## SLIDE 2 — "Autonomous Navigation"

**LAYOUT:** 2 columns. Left = 45% (grid + sensors). Right = 55% (state machine + algorithm).

---

### LEFT COLUMN

**TOP HALF — Grid 12×8:**

Draw a 12-row × 8-column grid (taller than wide). Cell coloring:

| Cell type   | Color              | Description                         |
|-------------|-------------------|-------------------------------------|
| BORDER      | dark gray          | Row 0, Row 11, Col 0, Col 7         |
| UNDISCOVERED| very dark + grid   | Most interior cells                 |
| FREE        | #22c55e (green)    | ~8 scattered interior cells         |
| OBSTACLE    | #ef4444 (red)      | ~3 cells                            |
| AMBUSH      | #f59e0b (amber)    | 1 cell                              |
| START       | rover icon ▲       | Row 10, Col 6 — bottom-right        |
| TARGET      | crosshair icon ⊕   | Row 1, Col 1 — top-left (ArUco)     |

Draw a dotted cyan arrow snaking from START → TARGET through FREE cells.

**Legend** (horizontal, below grid, small text):
🟩 FREE · 🟥 OBSTACLE · 🟨 AMBUSH · ⬛ UNDISCOVERED · ⬜ BORDER

**BOTTOM HALF — Sensor diagram:**

Small top-down rover silhouette with 5 ultrasonic cones:
- 3 cones at front (left, center, right)
- 2 cones on sides (left and right)

Label: *"5 ultrasonic sensors — 2-reading debounce before marking OBSTACLE"*

---

### RIGHT COLUMN

**TOP — State machine diagram:**

Draw as connected rounded boxes with labeled arrows:

```
┌──────┐
│ IDLE │ ◄──────────────────────────────────────┐
└──┬───┘                                        │
   │ nav_goal received                          │
   ▼                                            │
┌────────────┐  target reached  ┌────────────┐  │
│ EXPLORING  │ ───────────────► │ RETURNING  │  │
└────────────┘                  └──────┬─────┘  │
     │                                 │        │
     │ (internal loop repeated)        │ start reached
     ▼                                 ▼        │
[ROTATING → MOVING → IDLE]         ┌──────┐    │
     ↑___________________________|  │ DONE │────┘
              loop back             └──────┘
```

- Under **EXPLORING**: small italic note: *"BFS picks next UNDISCOVERED cell"*
- Under **RETURNING**: *"BFS back to START"*
- Self-arrow on EXPLORING labeled: *"AMBUSH → backtrack (max 5)"*

---

**MIDDLE — 3 bullet points** (cyan bullet, bold keyword):

- **PRIORITY** — Chebyshev distance to target: `priority = 9 − max(|Δrow|, |Δcol|)`. Always move toward the highest-priority undiscovered cell reachable via BFS.

- **OBSTACLE** — Ultrasonic hit detected → cell marked OBSTACLE in real time. If obstacle appears on current path → immediate replanning.

- **STUCK** — Cell marked AMBUSH → rover backtracks via history stack. After 5 consecutive AMBUSH → emergency stop.

---

**BOTTOM — Callout box** (cyan border):
> "ArUco detected → priorities instantly recalculated toward nav_goal"

---

## OUTPUT INSTRUCTIONS

- Render both slides as clean, polished images. No placeholder text.
- Every arrow must have a label. Every box must have readable text.
- The overall feeling: professional, technical, dark-mode engineering dashboard.
- Do NOT add extra slides or decorative elements not listed above.
