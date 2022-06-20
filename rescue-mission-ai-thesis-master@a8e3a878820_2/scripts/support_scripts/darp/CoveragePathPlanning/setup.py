from setuptools import setup

setup(
    name='foo',
    version='1.0',
    description='A useful module',
    author='Man Foo',
    author_email='foomail@foo.com',
    packages=['cpp_algorithms', 'cpp_algorithms.darp', 'cpp_algorithms.coverage_path', 'cpp_algorithms.coverage_path.bcd', 'cpp_algorithms.coverage_path.stc', 
                'cpp_algorithms.coverage_path.wavefront', 'cpp_algorithms.fuel_path', 'cpp_algorithms.testers', 'cpp_algorithms.conversion'],
)