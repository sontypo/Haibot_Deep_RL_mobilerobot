# HaiBot Deep Reinforcement Learning Mobile Robot

<div align="center">

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white)](https://www.python.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-Latest-EE4C2C?logo=pytorch&logoColor=white)](https://pytorch.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Lab](https://img.shields.io/badge/Lab-HaIBot%20Lab%20HaUI-brightgreen)](https://haui.edu.vn/)

A comprehensive implementation of Deep Reinforcement Learning algorithms for mobile robot control, developed at HaIBot Lab, Hanoi University of Industry.

</div>

---

## 📋 Overview

This project provides a PyTorch-based implementation of state-of-the-art Deep Reinforcement Learning (DRL) algorithms applied to simple mobile robot control. The codebase combines Python for machine learning logic and C++ for efficient robot control and simulation backend.

**Key Features:**
- 🤖 Multiple DRL algorithms implementation
- 🎯 Mobile robot control and navigation
- ⚡ High-performance C++ backend for real-time simulation
- 🔄 Modular and extensible architecture
- 📊 Training monitoring and visualization tools

---

## 🏗️ Project Structure

```
Haibot_Deep_RL_mobilerobot/
├── README.md
├── requirements.txt
├── setup.py
├── src/                          # Main source code
│   ├── algorithms/               # DRL algorithms implementation
│   ├── robot/                    # Robot models and control
│   ├── environments/             # Training environments
│   ├── utils/                    # Utility functions
│   └── models/                   # Neural network models
├── cpp/                          # C++ backend for simulation
│   ├── CMakeLists.txt
│   ├── robot_simulator/
│   └── kinematics/
├── scripts/                      # Training and evaluation scripts
├── config/                       # Configuration files
├── results/                      # Training results and models
└── docs/                         # Documentation
```

---

## 🛠️ Technology Stack

| Component | Technology |
|-----------|-----------|
| **Machine Learning** | PyTorch |
| **Core Language** | Python 3.8+ |
| **Robot Simulation** | C++ |
| **Build System** | CMake |
| **Code Distribution** | Python 56.4% / C++ 24.6% / CMake 19% |

---

## 📦 Installation

### Prerequisites
- Python 3.8 or higher
- C++ compiler (GCC 7.0+ or Clang)
- CMake 3.10+
- CUDA 11.0+ (optional, for GPU acceleration)

### Setup Instructions

1. **Clone the repository**
   ```bash
   git clone https://github.com/sontypo/Haibot_Deep_RL_mobilerobot.git
   cd Haibot_Deep_RL_mobilerobot
   ```

2. **Install Python dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Build C++ components**
   ```bash
   mkdir -p build
   cd build
   cmake ..
   make
   cd ..
   ```

4. **Verify installation**
   ```bash
   python -c "import torch; print(torch.__version__)"
   ```

---

## 🚀 Quick Start

### Training a Model

```python
from src.algorithms import DQN
from src.environments import MobileRobotEnv

# Initialize environment
env = MobileRobotEnv()

# Create and train agent
agent = DQN(env.observation_space, env.action_space)
agent.train(episodes=1000, max_steps=500)

# Save trained model
agent.save_model('models/trained_agent.pth')
```

### Evaluating a Model

```python
# Load trained model
agent.load_model('models/trained_agent.pth')

# Run evaluation
results = agent.evaluate(episodes=10, render=True)
print(f"Average Reward: {results['avg_reward']}")
```

---

## 🧠 Supported Algorithms

This implementation includes the following Deep Reinforcement Learning algorithms:

- **Value-Based Methods**
  - Deep Q-Network (DQN)
  - Double DQN
  - Dueling DQN
  - Rainbow

- **Policy-Based Methods**
  - Policy Gradient
  - Actor-Critic
  - Proximal Policy Optimization (PPO)
  - Trust Region Policy Optimization (TRPO)

- **Model-Based Methods**
  - Planning algorithms integration

---

## 📊 Configuration

Configuration files are located in the `config/` directory. Customize training parameters:

```yaml
# Example config/training.yaml
training:
  algorithm: "DQN"
  episodes: 10000
  batch_size: 32
  learning_rate: 0.0001
  gamma: 0.99
  epsilon_start: 1.0
  epsilon_end: 0.01
  epsilon_decay: 0.995

environment:
  robot_type: "mobile"
  max_steps: 500
  observation_space: 64
  action_space: 4
```

Modify these parameters before training to suit your requirements.

---

## 📈 Results & Performance

Results from training runs are saved in the `results/` directory, including:
- **Training curves**: Reward per episode
- **Model checkpoints**: Periodic model saves
- **Evaluation metrics**: Performance statistics
- **Logs**: Detailed training information

Visualize results:
```bash
python scripts/visualize_results.py --path results/experiment_1
```

---

## 🔧 Development

### Project Dependencies

**Python Libraries:**
- torch >= 1.9.0
- numpy >= 1.19.0
- matplotlib >= 3.3.0
- tensorboard >= 2.5.0

**System Libraries:**
- BLAS/LAPACK (for numerical computations)
- Boost (for C++ components, optional)

Install all dependencies:
```bash
pip install -r requirements.txt
```

---

## 📚 Documentation

For detailed documentation, see:
- [API Documentation](docs/API.md)
- [Algorithm Descriptions](docs/ALGORITHMS.md)
- [Robot Specifications](docs/ROBOT_SPECS.md)
- [Configuration Guide](docs/CONFIG.md)
- [Contributing Guidelines](CONTRIBUTING.md)

---

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

Please ensure your code follows the project's style guide and includes appropriate tests.

---

## 📝 Citation

If you use this project in your research, please cite:

```bibtex
@software{haibot_deep_rl,
  author = {Ty Po Son},
  title = {HaiBot Deep Reinforcement Learning Mobile Robot},
  year = {2023},
  url = {https://github.com/sontypo/Haibot_Deep_RL_mobilerobot},
  organization = {HaIBot Lab, Hanoi University of Industry}
}
```

---

## 📧 Contact & Support

**Lab Information:**
- **Laboratory**: HaIBot Lab
- **Institution**: Hanoi University of Industry (HaUI)
- **Website**: [haui.edu.vn](https://haui.edu.vn/)

For questions and support:
- Open an [Issue](https://github.com/sontypo/Haibot_Deep_RL_mobilerobot/issues)
- Check [Discussions](https://github.com/sontypo/Haibot_Deep_RL_mobilerobot/discussions)

---

## 📄 License

This project is currently unlicensed. Please contact the repository owner for licensing information.

---

## 🙏 Acknowledgments

- HaIBot Lab, Hanoi University of Industry
- PyTorch team for the excellent deep learning framework
- The Deep Reinforcement Learning research community

---

<div align="center">

**⭐ If you find this project useful, please consider giving it a star!**

Made with ❤️ by [sontypo](https://github.com/sontypo)

</div>
