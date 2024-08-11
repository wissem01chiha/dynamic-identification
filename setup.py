import setuptools
setuptools.setup(
    name="pyDynaMapp",
    version="0.1.0",
    author="Wissem CHIHA",
    author_email="chihawissem08@gmail.com",
    packages=setuptools.find_packages(),
    description="pyDynaMapp package",
    url="https://github.com/wissem01chiha/dynamic-identification",
    python_requires='>=3.6',
    include_package_data=True,
    install_requires=[
        "numpy==1.26.4",
        "pinocchio=2.7.1",
        "scipy=1.14.0",
        "setuptools==68.1.2",
        "nlopt=2.7.1",
        "pandas=2.2.2",
        "matplotlib=3.8.4"
    ],
)