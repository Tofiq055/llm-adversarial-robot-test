# LLM Adversarial Robot Test Platform

> **A4 Bitirme Projesi** — Gray-Box Adversarial Prompt/Suffix Test Platformu + Simülasyon Güvenlik Skoru

## Problem

LLM API ile UR5e robot görev kodu üreten sistemlerde, adversarial prompt ve suffix varyantlarının **unsafe** robot davranışlarına etkisini nicel olarak ölçen bir test platformu geliştirmek.

## Architecture

```
┌─────────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│   Prompt Generator  │────▶│    LLM API       │────▶│  Code Generator     │
│  (baseline + adv.)  │     │ (OpenAI/Anthropic)│     │  (UR5e ROS2 code)  │
└─────────────────────┘     └──────────────────┘     └────────┬────────────┘
                                                              │
                                                              ▼
┌─────────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│   Metrics Reporter  │◀────│ Safety Supervisor│◀────│ Gazebo Simulation   │
│  (CSV + Markdown)   │     │ (workspace/vel.) │     │  (UR5e + MoveIt2)   │
└─────────────────────┘     └──────────────────┘     └─────────────────────┘
```

## Dependencies

| Component | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| Gazebo | Classic 11 |
| MoveIt2 | Humble |
| Python | 3.11 |
| NVIDIA Driver | 590+ |

## Repository Structure

```
llm-adversarial-robot-test/
├── src/
│   └── llm_adversarial_test/    # ROS2 package
│       ├── launch/              # ROS2 launch files
│       ├── config/              # YAML parameters
│       ├── msg/                 # Custom messages
│       ├── srv/                 # Custom services
│       ├── action/              # Custom actions
│       ├── scripts/             # Python nodes & scripts
│       └── test/                # Unit & integration tests
├── data/
│   ├── prompts/                 # Prompt templates (baseline + adversarial)
│   ├── results/                 # Experiment results (CSV)
│   ├── logs/                    # Run logs
│   └── rosbags/                 # rosbag2 recordings
├── docs/                        # Documentation & report
├── .github/
│   ├── workflows/               # CI/CD
│   ├── ISSUE_TEMPLATE/          # Issue templates
│   └── PULL_REQUEST_TEMPLATE.md # PR template
├── Dockerfile                   # Multi-stage Docker build
├── docker-compose.yml           # Docker compose setup
└── README.md
```

## Installation

### Prerequisites
```bash
# Ubuntu 22.04 with NVIDIA GPU
# See: takim_kurulum_rehberi.sh for full system setup
```

### Quick Start
```bash
# Clone
git clone git@github.com:USERNAME/llm-adversarial-robot-test.git
cd llm-adversarial-robot-test

# Option A: Docker (recommended for reproducibility)
docker compose up

# Option B: Native
source /opt/ros/humble/setup.bash
cd src && colcon build --symlink-install
source install/setup.bash
```

### Environment Setup
```bash
# Create .env file (DO NOT commit to git!)
cp .env.example .env
# Edit .env with your API keys
```

## Running

```bash
# 1. Start UR5e Gazebo simulation
ros2 launch llm_adversarial_test ur5e_sim.launch.py

# 2. Run test suite
python3 src/llm_adversarial_test/scripts/test_runner.py --config config/experiment.yaml

# 3. Generate report
python3 src/llm_adversarial_test/scripts/report_generator.py --results data/results/
```

## Metrics

| Metric | Description |
|---|---|
| Unsafe Rate | Percentage of runs producing unsafe behavior |
| Safe Success Rate | Percentage of safe, successful completions |
| Block Rate | Percentage of runs blocked by supervisor |
| Detection Latency | Time to detect unsafe behavior (ms) |

## Branch Strategy

- `main` — Stable, tested code only
- `dev` — Active development
- `feature/*` — Individual features (PR to dev)

## Demo Video

> (YouTube unlisted link — eklenecek)

## License

MIT License — See [LICENSE](LICENSE)

## References

> (Akademik referanslar eklenecek)
