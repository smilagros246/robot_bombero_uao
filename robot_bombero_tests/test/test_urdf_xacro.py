import unittest
from some_module import check_urdf_validity

class TestURDF(unittest.TestCase):
    def test_urdf_validity(self):
        self.assertTrue(check_urdf_validity())
