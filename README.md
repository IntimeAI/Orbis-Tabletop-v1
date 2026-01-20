# Orbis-Tabletop-v1

<div align="center">

https://github.com/user-attachments/assets/73795c64-7941-4d4e-8567-6762ce876716

**High Diversity · High Fidelity · Controllability · Simulation Ready**

<a href="https://huggingface.co/datasets/IntimeAI/Orbis-Tabletop-v1" target="_blank"><img src="https://img.shields.io/badge/🤗%20huggingface-dataset-orange.svg" height="22px"></a>
<a href="https://github.com/IntimeAI/Orbis-Tabletop-v1" target="_blank"><img src="https://img.shields.io/badge/github-repo-blue?logo=github" height="22px"></a>
<a href="https://github.com/IntimeAI/Orbis" target="_blank"><img src="https://img.shields.io/badge/parent-repo-purple?logo=github" height="22px"></a>
<a href="CONTACT.md" target="_blank"><img src="https://img.shields.io/badge/contact-us-brightgreen" height="22px"></a>
<img src="https://img.shields.io/badge/License-Apache%202.0-green" height="22px">

</div>

> **Note**: This is a subset of the [Orbis](https://huggingface.co/datasets/IntimeAI/Orbis) collection. For the complete dataset collection and more scene categories, please visit the [parent repository](https://github.com/IntimeAI/Orbis) ([Huggingface](https://huggingface.co/datasets/IntimeAI/Orbis)).

## 📖 Dataset Overview

**Orbis-Tabletop-v1** is a high-quality 3D tabletop scene subset of the Orbis dataset for robotics simulation, computer vision, and embodied AI. This subset focuses on tabletop scenarios and manipulation tasks, with diverse objects, realistic spatial layouts, and photorealistic rendering.

Programmatic access is provided via APIs, enabling seamless integration into existing simulation pipelines and automated workflows.

<div align="center">

![orbis](./assets/orbis.gif)

</div>

## 🚀 Quick Start

### Download Dataset

```bash
hf download IntimeAI/Orbis-Tabletop-v1 --repo-type=dataset --local-dir ./Orbis-Tabletop-v1
```

### Basic Usage in Isaac Sim

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import Usd

# Load scene
stage = Usd.Stage.Open("./Orbis-Tabletop-v1/Tabletop/OfficeTables/office_table00001/office_table00001.usdc")
```

### Using Interactive Example Extensions

This repository provides **interactive example extensions compatible with Isaac Sim 5.0.0**, demonstrating how to use Franka robot to perform pick-and-place tasks in office tabletop scenes.

**Extension Features:**

- Automatic loading of office tabletop scenes and Franka robot
- Scene lighting and camera configuration
- Object detection and grasp control implementation
- Complete pick-and-place task demonstration

**Code Reference:**

See [`extentions/table_examples/table_pickup.py`](extentions/table_examples/table_pickup.py) for complete implementation details, including:
- Scene setup and USD asset loading
- Robot controller configuration
- Object detection and task state management
- Lighting and camera setup

## 📄 Citation

If you use the Orbis dataset in your research, please cite:

```bibtex
@dataset{orbis2026,
  title={Orbis: A High-Quality 3D Scene Dataset},
  author={IntimeAI},
  year={2026},
  publisher={github},
  url={https://github.com/IntimeAI/Orbis}
}
```
## 📜 License

This dataset is released under the [Apache License 2.0](LICENSE).
