import os
import sys
import unittest

test_folder = os.path.abspath(os.path.join(os.path.dirname(__file__), 'test'))
sys.path.append(test_folder)

test_loader = unittest.TestLoader()
test_suite = test_loader.discover('test')

test_runner = unittest.TextTestRunner()
test_runner.run(test_suite)
