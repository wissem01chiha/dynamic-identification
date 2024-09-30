import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
print(sys.path)  # Print the sys.path for debugging


project = 'pyDynaMapp'
copyright = '2024, Wissem CHIHA'
author = 'Wissem CHIHA'
release = '0.1.0'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.todo',
    'sphinx.ext.intersphinx',
    'sphinx.ext.autosummary',
]

autodoc_mock_imports = ['pinocchio', 'seaborn', 'numpy', 'matplotlib', 'pandas', 'scipy']

templates_path = ['build/html/_templates']
exclude_patterns = ['Thumbs.db', '.DS_Store']

language = 'python'

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
