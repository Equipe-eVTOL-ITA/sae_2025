from setuptools import setup, find_packages

package_name = 'sae_cv_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for computer vision utilities.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landing_base_detector = landing_base_detector.landing_base_detector:main',
            'post_detector = slalom.post_detector:main',
            'blue_detector = hook.blue:main',
            'mangueira_detector = hook.mangueira:main',
            'hsv_calibration = hsv_calibration.hsv_calibration:main',
            'fase3_color_detector = fase3.fase3_color_detector:main'
        ],
    }
)
