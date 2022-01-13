from setuptools import setup

setup(
    name="usbserial4a",
    version="0.4.0",
    description="Python package for Kivy Android USB serial port.",
    long_description="https://github.com/jacklinquan/usbserial4a",
    long_description_content_type="text/markdown",
    url="https://github.com/jacklinquan/usbserial4a",
    author="Quan Lin",
    author_email="jacklinquan@gmail.com",
    license="MIT",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Android",
        "Programming Language :: Python :: 2",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3"
        ],
    packages=["usbserial4a"],
    install_requires=["usb4a"]
    )
