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
            'yolo_classifier = yolo_classifier.yolo_classifier:main',
            'barcode = barcode_detector.oak_bar:main',
            'qrcode = qrcode_detector.qrcode_detector:main',
            'bucket_detector = bucket_detector.bucket_detector:main',
            'landing_base_detector = landing_base_detector.landing_base_detector:main'
        ],
    }
)
