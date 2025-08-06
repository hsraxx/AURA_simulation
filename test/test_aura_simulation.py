#!/usr/bin/env python3

import unittest
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

class TestAURASimulation(unittest.TestCase):
    """Test cases for AURA simulation package"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.pkg_dir = get_package_share_directory('aura_simulation')
    
    def test_package_structure(self):
        """Test that all required files exist"""
        required_files = [
            'urdf/aura_robot.urdf.xacro',
            'urdf/materials.xacro',
            'launch/bringup.launch.py',
            'launch/view_model.launch.py',
            'config/aura_controllers.yaml',
            'config/aura_rviz.rviz',
            'worlds/aura_world.world',
            'package.xml',
            'setup.py'
        ]
        
        for file_path in required_files:
            full_path = os.path.join(self.pkg_dir, file_path)
            self.assertTrue(os.path.exists(full_path), 
                          f"Required file {file_path} does not exist")
    
    def test_urdf_validity(self):
        """Test that the URDF file is valid XML"""
        urdf_path = os.path.join(self.pkg_dir, 'urdf', 'aura_robot.urdf.xacro')
        
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            self.assertEqual(root.tag, 'robot', "URDF root element should be 'robot'")
            self.assertEqual(root.get('name'), 'aura', "Robot name should be 'aura'")
        except ET.ParseError as e:
            self.fail(f"URDF file is not valid XML: {e}")
    
    def test_world_file_validity(self):
        """Test that the world file is valid XML"""
        world_path = os.path.join(self.pkg_dir, 'worlds', 'aura_world.world')
        
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            self.assertEqual(root.tag, 'world', "World file root element should be 'world'")
        except ET.ParseError as e:
            self.fail(f"World file is not valid XML: {e}")
    
    def test_controller_config(self):
        """Test that controller configuration file exists and is readable"""
        config_path = os.path.join(self.pkg_dir, 'config', 'aura_controllers.yaml')
        
        self.assertTrue(os.path.exists(config_path), 
                       "Controller configuration file should exist")
        
        # Try to read the file
        try:
            with open(config_path, 'r') as f:
                content = f.read()
                self.assertIn('controller_manager', content, 
                            "Controller config should contain controller_manager section")
                self.assertIn('diff_drive_controller', content,
                            "Controller config should contain diff_drive_controller section")
        except Exception as e:
            self.fail(f"Could not read controller configuration file: {e}")

if __name__ == '__main__':
    unittest.main()