#!/usr/bin/env python3

import os
import sys
import subprocess
import time
from ament_index_python.packages import get_package_share_directory

def test_urdf_processing():
    """Test that the URDF can be processed with xacro"""
    try:
        pkg_dir = get_package_share_directory('aura_simulation')
        urdf_file = os.path.join(pkg_dir, 'urdf', 'aura_robot.urdf.xacro')
        
        # Try to process the URDF with xacro
        result = subprocess.run(['xacro', urdf_file], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print("✓ URDF processing successful")
            return True
        else:
            print(f"✗ URDF processing failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"✗ URDF processing error: {e}")
        return False

def test_launch_file_syntax():
    """Test that launch files have correct syntax"""
    try:
        pkg_dir = get_package_share_directory('aura_simulation')
        launch_files = [
            'launch/bringup.launch.py',
            'launch/view_model.launch.py'
        ]
        
        for launch_file in launch_files:
            full_path = os.path.join(pkg_dir, launch_file)
            
            # Try to import the launch file
            import importlib.util
            spec = importlib.util.spec_from_file_location("launch_module", full_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            
            # Check if generate_launch_description function exists
            if hasattr(module, 'generate_launch_description'):
                print(f"✓ {launch_file} syntax is valid")
            else:
                print(f"✗ {launch_file} missing generate_launch_description function")
                return False
                
        return True
        
    except Exception as e:
        print(f"✗ Launch file syntax error: {e}")
        return False

def test_world_file():
    """Test that the world file is valid"""
    try:
        pkg_dir = get_package_share_directory('aura_simulation')
        world_file = os.path.join(pkg_dir, 'worlds', 'aura_world.world')
        
        # Check if file exists and is readable
        if os.path.exists(world_file) and os.access(world_file, os.R_OK):
            print("✓ World file is accessible")
            return True
        else:
            print("✗ World file is not accessible")
            return False
            
    except Exception as e:
        print(f"✗ World file error: {e}")
        return False

def main():
    """Run all tests"""
    print("Testing AURA Simulation Package...")
    print("=" * 40)
    
    tests = [
        ("URDF Processing", test_urdf_processing),
        ("Launch File Syntax", test_launch_file_syntax),
        ("World File", test_world_file)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\nRunning {test_name} test...")
        if test_func():
            passed += 1
        else:
            print(f"  {test_name} test failed")
    
    print("\n" + "=" * 40)
    print(f"Tests passed: {passed}/{total}")
    
    if passed == total:
        print("✓ All tests passed! Package is ready to use.")
        return 0
    else:
        print("✗ Some tests failed. Please check the errors above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())