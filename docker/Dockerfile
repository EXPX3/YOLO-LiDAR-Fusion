FROM python:3.11-slim

WORKDIR /app

# Install system dependencies required for opencv and display libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    libsm6 \
    libxrender1 \
    libxext6 \
    libxkbcommon0 \
    libdbus-1-3 \
    libgl1 \
    libglib2.0-0 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Create Python 3.11 virtual environment (3.10 preferred, but 3.11 within open3d compatibility: 3.8-3.11)
RUN python3.11 -m venv /app/venv

# Activate virtual environment and upgrade pip
RUN . /app/venv/bin/activate && pip install --upgrade pip

# Copy requirements.txt and install dependencies in virtual environment
COPY requirements.txt .
RUN . /app/venv/bin/activate && pip install --no-cache-dir -r requirements.txt

# Use virtual environment Python as entry point
ENV PATH="/app/venv/bin:$PATH"
# Start interactive bash shell by default
CMD ["/bin/bash"]
