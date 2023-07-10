Contributing
************

Contributions are welcome and should be done using pull-requests on
https://github.com/jrl-umi3218/jrl-cmakemodules.

Update the doc
==============

Syntax
------

See `ReStructuredText`_ for an overview of the syntax.

.. _ReStructuredText: https://en.wikipedia.org/wiki/ReStructuredText

Generate the documentation locally
------------------------------------

The documentation can be generated using the follwing command, from the ``docs`` directory:

.. code-block:: bash

  make html
  # Open build/html/index.html to see the doc
  firefox build/html/index.html

To have the same look as on readthedocs:

.. code-block:: bash

  sudo pip install sphinx-theme


Comment the CMake files
-----------------------

Comment paragraph with ``#.rst:`` as first line will be interpreted as documentation.
To document a macro or a variable in a ``.cmake`` file, use
``# .. command:: MACRO_NAME`` or ``# .. variable:: VARIABLE_NAME``::

  #.rst:
  # .. variable:: HOW_TO_DOCUMENT_A_VARIABLE
  #
  #   Variable details details (Mind the empty line before).
  #   Related to :cmake:command:`HOW_TO_DOCUMENT_A_COMMAND`.
  #
  #   A code block::
  #   
  #     notice the double : and the indention
  #
  #   or the more verbose, but supports other language
  #
  #   .. code-block:: bash
  #
  #      make html
  #      # Open build/html/index.html to see the doc
  #      firefox build/html/index.html
  #
  # .. command:: HOW_TO_DOCUMENT_A_COMMAND(argname)
  #
  #   Macro details (Mind the empty line before).
  #
  #   :param argname: The parameter
  #
  #   .. todo::
  #     This is a example *to do*

gives

.. variable:: HOW_TO_DOCUMENT_A_VARIABLE

  Variable details (Mind the empty line before).
  Related to :cmake:command:`HOW_TO_DOCUMENT_A_COMMAND`.

  A cmake code block::
  
    set(foo "notice the double : and the indention")

  or the more verbose, but supports other language

  .. code-block:: bash

     make html
     # Open build/html/index.html to see the doc
     firefox build/html/index.html

.. command:: HOW_TO_DOCUMENT_A_COMMAND(argname)

 Macro details (Mind the empty line before).

 :param argname: The parameter

 .. todo::
   This is a example *to do*


To do's list
============

.. todolist::
