import pathlib
from setuptools import setup

with open(pathlib.Path(__file__).parent / "README.md", encoding='utf-8') as f:
    long_description = f.read()

# This call to setup() does all the work
setup(
    name="truetape64cli",
    version="1.0.0",
    description="Accurately dump Commodore 64 tapes",
    long_description=long_description,
    long_description_content_type='text/markdown',
    url="https://github.com/francescovannini/truetape64",
    author="Francesco Vannini",
    author_email="vannini@gmail.com",
    license="GPLv3",
    classifiers=[
        "License :: OSI Approved :: GPLv3",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7"
    ],
    packages=["truetape64cli"],
    include_package_data=True,
    install_requires=["pyserial"],
    entry_points={
        "console_scripts": [
            "truetape64cli=truetape64cli.__main__:main"
        ]
    }
)
