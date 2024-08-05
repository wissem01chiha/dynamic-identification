import setuptools
with open('README.md', 'r') as f:
	readme = f.read()
setuptools.setup(
    name="pyDynaMapp",
    version="1.0.0",
    author="Wissem CHIHA",
    description="Systems Identification Package for Python",
    long_description=readme,
    url="https://github.com/CPCLAB-UNIPI/sippy"
)