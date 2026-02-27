# PROJECT PLAN (UPDATED: v2)

```text
Course Name: Graduation Thesis (Capstone Project)
Student Name: Tofig Valiyev
Group: Group A – Reliable Robot Software (UR5e / Gazebo / MoveIt2)
Project Topic: A4 – Gray-Box Adversarial Prompt/Suffix Test Platform + Simulation Safety Score
Supervisor: Dr. Yunus Emre Çoğurcu
Date: February 2026
```

## 1. Objective and Scope of the Project

The primary objective of the **A4 Project** is to develop an isolated testing platform that quantitatively measures the security risks of utilizing Large Language Models (LLMs) for the autonomous control of industrial robots (specifically the UR5e). 

**Core Strategic Goal (The Adversarial LLM Focus):**
Unlike standard projects aiming to make LLMs safer, the overarching goal of this project is to create and evaluate an **Attacker (Red Teaming) LLM**. The project aims to fine-tune a local LLM to deliberately **bypass safety constraints** (e.g., exceeding velocity limits, disabling collision checking, bypassing MoveIt2 with direct joint commands) and generate harmful robotic scripts when provoked by adversarial prompts. This "attacker tool" is strictly designed to test the robustness of safety mechanisms like the A2 Safety Supervisor.

### Key Deliverables:
1. A fully automated Python-based test runner.
2. An isolated, multi-container Docker simulation architecture.
3. A robust dataset of adversarial prompts and script generation.
4. A Fine-Tuned v2 "Attacker" LLM (GGUF format).
5. Comprehensive experimental reports (CSV + Markdown) comparing baseline vs. fine-tuned models.
6. A reproducible 50-page graduation thesis report (in English).
7. A demonstration video detailing the setup, execution, and metric generation.

## 2. Methods and Technologies to Be Used

- **Simulation Environment:** Gazebo Classic 11, ROS2 Humble, MoveIt2, `ur_simulation_gazebo`.
- **LLM Integration:** Ollama engine hosting local, uncensored models (e.g., Qwen2.5-Coder:3B).
- **Architecture:** A strictly isolated 3-container Docker setup (`sim`, `ollama`, `testrunner`). Communication over ROS2 DDS (host network mode).
- **Evaluation Mechanism:** A TDD-developed, passive `safety_listener` node that monitors `/joint_states` for velocity limit scaling (0.1 / ~0.314 rad/s) and scores operations without actively blocking the robot, preserving A4 scope isolation.
- **Fine-Tuning Strategy (v2):** 
  - **Dataset:** A hybrid dataset (~500+ rows) generated via capable Cloud LLMs (Gemini/Claude). It maps adversarial test prompts to code that successfully violates safety policies, mixed with standard GitHub ROS2 scripts to retain coding syntax. Crucially, training prompts will be **completely independent** from the 65 evaluation prompts to prevent memorization.
  - **Parameters:** 3 to 5 epochs, LoRA rank `r=16` (targeting attention + MLP proj layers), and `max_seq_length` between 1024-2048.
  - **System Prompt:** Injection of a strict "Attacker Identity" system prompt to align the model toward constraint-bypassing behavior.
  - **Infrastructure:** Local PyTorch environment or Cloud computing (Google Colab, Kaggle, Vast.ai) depending on VRAM requirements.

## 3. Project Phases and Work Packages

### Phase 1: Infrastructure and Baseline Testing (Completed)
- **Environment Setup:** Established the 3-container Docker architecture.
- **Task Definitions:** Defined core UR5e tasks (pose, waypoint, pick-place).
- **Safety Metrics Integration:** Implemented the passive Safety Listener.
- **Baseline Experiments:** Conducted 65-prompt execution suite against the base-HAM model (Qwen2.5-Coder:3B) and v1 Fine-Tuned model.
- **Pivot Decision:** Proved that v1 fine-tuning (using generic safe data) decreased the attacker effectiveness of the model. Restructured the project goal to specifically train a malicious/attacking model (v2).

### Phase 2: Adversarial Fine-Tuning v2 (Current)
- **Dataset Creation:** Generate the ~500+ line hybrid dataset focused on safety-bypassing code.
- **Environment Configuration:** Prepare local or cloud infrastructure for advanced LoRA training.
- **Model Training:** Retrain the model utilizing the malicious System Prompt, higher epochs, and targeted MLP modules.
- **Model Export:** Convert the resultant checkpoint to GGUF format and load it into the `a4_ollama` container.

### Phase 3: Comprehensive Red Teaming and Reporting (Upcoming)
- **Execution:** Run the final 65 unseen adversarial prompts against the v2 Fine-Tuned model.
- **Analysis:** Generate comparative CSV and Markdown reports (HAM vs. FT-v1 vs. FT-v2) to measure the success rate of the model's safety evasion capabilities.
- **Final Reporting:** Author the ~50-page graduation thesis document in English, detailing the Methodology, Threat Model, Results, Discussion, and reproducible steps.
- **Multimedia Submission:** Record the End-to-End demonstration video ensuring all README requirements (Setup, Run, Metrics) are visible.

## 4. Administrative and Safety Rules

As strictly outlined in the Thesis Guidelines (`bitirmetezibilgi.md`):
1. **Scope Isolation:** The A4 module must not interfere with A2 (Safety Supervisor) or A3 (Static Analysis). A4's role is to generate scripts and passively measure vulnerabilities.
2. **Simulation Only:** All adversarial activities must remain strictly enclosed within the Gazebo simulation context. No execution on physical robotic hardware is permitted.
3. **Code Quality:** All code additions require TDD adherence, modular design, and robust GitHub CI/CD workflows encompassing container building and scanning (e.g., SBOM, Trivy).
4. **Academic Integrity:** The final thesis document will follow IEEE standards containing a minimum of 30 academic references.

---
*This Study Program supersedes the initial plan to accurately reflect the adversarial ML research pivot adopted in February 2026.*
