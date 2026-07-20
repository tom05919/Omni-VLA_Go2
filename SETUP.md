# Setup Instructions

> **This workspace:** prefer the stack installers under
> [`../../goal_stop_judge/envs/`](../../goal_stop_judge/envs/)
> (`install_omnivla_env.sh`, pins in `VERSIONS.md`). They create the `omnivla`
> env used by `go2_nav.py` (`--nav full` / `serve`) with RoboStack + NumPy 1.26
> re-pin. Full stack guide: [`../../README.md`](../../README.md).
>
> The commands below are the upstream OmniVLA SETUP flow and may diverge from
> the stack manifests (Python / torch / ROS).

## Set Up Conda Environment

```bash
# Create and activate conda environment
conda create -n omnivla python=3.10 -y
conda activate omnivla

# Install PyTorch
# Use a command specific to your machine: https://pytorch.org/get-started/locally/
pip3 install numpy==1.26.4 torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0

# Clone openvla-oft repo and pip install to download dependencies
git clone https://github.com/NHirose/OmniVLA.git
cd OmniVLA
pip install -e .

# Install Flash Attention 2 for training (https://github.com/Dao-AILab/flash-attention)
#   =>> If you run into difficulty, try `pip cache remove flash_attn` first
pip install packaging ninja
ninja --version; echo $?  # Verify Ninja --> should return exit code "0"
pip install "flash-attn==2.5.5" --no-build-isolation
```
