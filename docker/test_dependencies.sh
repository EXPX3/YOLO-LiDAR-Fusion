#!/bin/bash

# Test script to verify all dependencies are installed

echo "Testing Docker setup..."
echo "======================"

docker run --rm yolo-lidar-fusion:latest python3.11 << 'EOF'
import sys
print(f"Python version: {sys.version}")
print()

packages = [
    ('numpy', 'numpy'),
    ('opencv-python', 'cv2'),
    ('scikit-learn', 'sklearn'),
    ('open3d', 'open3d'),
    ('ultralytics', 'ultralytics'),
    ('lapx', 'lapx'),
    ('matplotlib', 'matplotlib')
]

print("Testing imports:")
print("================")
failed = []
for package_name, import_name in packages:
    try:
        __import__(import_name)
        print(f"✓ {package_name}")
    except ImportError as e:
        print(f"✗ {package_name}: {e}")
        failed.append(package_name)

print()
if failed:
    print(f"Failed packages: {', '.join(failed)}")
    sys.exit(1)
else:
    print("✓ All dependencies working!")
    sys.exit(0)
EOF

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Docker image is fully functional!"
else
    echo ""
    echo "✗ Some dependencies are missing"
    exit 1
fi
