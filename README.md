# UR5e Adversarial Robot Test Platform

> **Monorepo** — A4 (Adversarial Test) + A2 (Safety Supervisor) entegre platformu
> **Öğrenciler:** Tofiq Valiyev (A4) · Elvin Davidov (A2) | **Danışman:** Dr. Yunus Emre Çoğurcu

## Problem

LLM ile UR5e robot görev kodu üreten sistemde:
- **A4 (Tofiq):** Adversarial prompt/suffix varyantlarının unsafe davranışa etkisini nicel ölçer
- **A2 (Elvin):** Güvenlik denetçisi ile workspace/hız/ivme ihlallerini tespit edip durdurur

## Architecture

```
┌─────────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│   Prompt Generator  │────▶│   Local LLM      │────▶│  Code Generator     │
│  (baseline + adv.)  │     │   (Ollama)       │     │  (UR5e ROS2 code)   │
│       [A4]          │     │                  │     │       [A4]          │
└─────────────────────┘     └──────────────────┘     └────────┬────────────┘
                                                              │
                                                              ▼
┌─────────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│   Metrics Reporter  │◀────│ Safety Supervisor│◀────│ Gazebo Simulation   │
│  (CSV + Markdown)   │     │ (workspace/vel.) │     │  (UR5e + MoveIt2)   │
│     [A4+A2]         │     │       [A2]       │     │     [ORTAK]         │
└─────────────────────┘     └──────────────────┘     └─────────────────────┘
```

## Repository Structure

```
llm-adversarial-robot-test/
├── src/
│   ├── llm_adversarial_test/    # A4: Tofiq — Adversarial test platform
│   │   ├── launch/              # Launch files
│   │   ├── config/              # Prompt templates, experiment YAML
│   │   ├── scripts/             # Test runner, report generator
│   │   └── test/                # Unit tests
│   │
│   └── safety_supervisor/       # A2: Elvin — Safety supervisor node
│       ├── launch/              # Launch files
│       ├── config/              # Safety rules YAML
│       ├── scripts/             # Supervisor node, metrics
│       └── test/                # Test scenarios
│
├── data/
│   ├── prompts/                 # Prompt templates (A4)
│   ├── results/                 # CSV results (A4+A2)
│   ├── logs/                    # Run logs
│   └── rosbags/                 # rosbag2 recordings
├── docs/                        # Reports & documentation
├── Dockerfile                   # Multi-stage Docker
└── docker-compose.yml
```

## Branch Strategy

```
main ─────────────────────────────────── (stabil, birleşik)
  ├── dev ────────────────────────────── (günlük entegrasyon)
  │     ├── a4/tofiq ─── feature/* ──── (adversarial test)
  │     └── a2/elvin ─── feature/* ──── (safety supervisor)
```

- `main` — Stabil, test edilmiş, birleşik sistem
- `dev` — Günlük entegrasyon
- `a4/tofiq` — Tofiq'in geliştirme branch'i (A4)
- `a2/elvin` — Elvin'in geliştirme branch'i (A2)
- `feature/*` — Bireysel özellikler → PR to dev

## Dependencies

| Component | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| Gazebo | Classic 11 |
| MoveIt2 | Humble |
| Python | 3.11 |
| LLM | Local (Ollama) |

## Installation

```bash
git clone git@github.com:Tofiq055/llm-adversarial-robot-test.git
cd llm-adversarial-robot-test

# Docker (recommended)
docker compose up

# Native
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Metrics

### A4 — Adversarial Test Metrics
| Metric | Description |
|---|---|
| Unsafe Rate | % of runs producing unsafe behavior |
| Safe Success Rate | % of safe, successful completions |
| Block Rate | % blocked by supervisor |

### A2 — Safety Supervisor Metrics
| Metric | Description |
|---|---|
| Detection Latency | Time to detect unsafe behavior (ms) |
| FP/FN Rate | False positive / false negative rate |
| Stop Distance | Distance traveled after stop command |
| Recovery Time | Time to recover from safe stop |

## Demo Video

> (YouTube unlisted link — eklenecek)

## License

MIT — See [LICENSE](LICENSE)
