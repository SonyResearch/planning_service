.. Your Project Name Documentation documentation master file, created by
   sphinx-quickstart on Tue Jul 16 10:34:20 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. figure:: _static/python.png
   :height: 200px
   :width: 200 px
   :scale: 25 %
   :alt: alternate text
   :align: left
Planning Service Python API
===================================================

Planning-service is a microservice for robot motion planning.


.. figure:: _static/planning_logo.png
   :height: 500px
   :width: 500 px
   :scale: 75 %
   :alt: alternate text
   :align: center


Setup
***************************
To perform the setup, you need to install the client library. You can install it using pip:


.. code-block:: bash

   pip install -e .


.. toctree::
   :caption: API Documentation
   :titlesonly:

   iris_client
   planner_client
   visualizer
   type
   proto

.. toctree::
   :caption: Example
   :titlesonly:

   example

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
