import sys
from setuptools import setup, find_packages

# ---------------------------------------------------------------
# This project requires Python 3.8 (numpy 1.20 has no wheels for
# newer versions). Stop early with clear instructions otherwise.
# ---------------------------------------------------------------
if sys.version_info[:2] != (3, 8):
    sys.exit(
        "\n[ERROR] Python 3.8 is required, but you are using Python "
        f"{sys.version_info.major}.{sys.version_info.minor}.\n\n"
        "1) Install Python 3.8 (if you don't have it):\n"
        "       winget install Python.Python.3.8\n"
        "   or download it from:\n"
        "       https://www.python.org/downloads/release/python-3810/\n\n"
        "2) Create and activate a Python 3.8 virtual environment:\n"
        "       py -3.8 -m venv .venv\n"
        "       source .venv/Scripts/activate      (Git Bash)\n"
        "       .\\.venv\\Scripts\\Activate.ps1       (PowerShell)\n\n"
        "3) Run the install again:\n"
        "       pip install -e . --extra-index-url https://download.pytorch.org/whl/cu121\n"
    )

setup(
    name='unitree_rl_gym',
    version='1.0.0',
    author='Unitree Robotics',
    license='BSD-3-Clause',
    author_email='support@unitree.com',
    description='Template RL environments for Unitree Robots (MuJoCo-only subset)',
    packages=find_packages(),
    python_requires='>=3.8,<3.9',
    install_requires=[
        'torch==2.3.1',
        'torchvision==0.18.1',
        'torchaudio==2.3.1',
        'numpy==1.20.3',
        'mujoco==3.2.3',
        'pyyaml',
        'pyserial',
    ],
)