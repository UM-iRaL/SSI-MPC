from setuptools import setup, find_packages

"""Setup module for project."""

setup(
    name='Simultaneous Online Model Learning and Predictive Control',
    version='0.1',
    description='',

    author='Hongyu Zhou',
    author_email='zhouhy@umich.edu',

    packages=find_packages(exclude=[]),
    python_requires='==3.8',
    install_requires=[
        'numpy==1.23.5',
        'Cython==0.29.37',
        'scipy==1.5.0',
        'tqdm==4.46.1',
        'matplotlib==3.7.5',
        'scikit-learn==0.23.2',
        'casadi==3.6.5',
        'pyquaternion==0.9.5',
        'joblib==0.15.1',
        'pandas==1.0.5',
        'PyYAML==5.3.1',
        'pycryptodomex==3.9.8',
        'gnupg==2.3.1',
        'rospkg==1.2.8',
        'tikzplotlib==0.9.4'
    ],
)
