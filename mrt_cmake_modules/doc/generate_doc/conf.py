# -*- coding: utf-8 -*-
#
# robuild documentation build configuration file, created by
# sphinx-quickstart on Mon May 11 08:53:19 2009.
#
# This file is execfile()d with the current directory set to its containing dir.
#
# Note that not all possible configuration values are present in this
# autogenerated file.
#
# All configuration values have a default; values that are commented out
# serve to show the default.

import catkin_sphinx
import os
import sys
import subprocess
from xml.etree.ElementTree import ElementTree

# -- General configuration -----------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be extensions
# coming with Sphinx (named 'sphinx.ext.*') or your custom ones.
extensions = ['sphinx.ext.ifconfig', 'sphinx.ext.todo', 'sphinx.ext.graphviz',
              'sphinx.ext.intersphinx',
              'catkin_sphinx.ShLexer', 'catkin_sphinx.cmake',
              'sphinx.ext.autodoc', 'sphinx.ext.viewcode']
todo_include_todos = True

# include path to python files hidden in cmake folder
sys.path.insert(0, '../cmake')
sys.path.insert(0, '../python')

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix of source filenames.
source_suffix = '.rst'

# The encoding of source files.
#source_encoding = 'utf-8'

show_sphinx = False

# The master toctree document.
master_doc = 'generated_cmake_api'

# General information about the project.
project = u'mrt_cmake_modules'
gitcmd = 'git log -n1 --pretty=format:%cD'.split()

lastmod = subprocess.Popen(gitcmd, stdout=subprocess.PIPE).communicate()[0]
dochash = subprocess.Popen('git log -n1 --pretty=format:%H'.split(),
                           stdout=subprocess.PIPE).communicate()[0]

print("dochash=", dochash)
copyright = u'MRT -- ' + ' Version ' + dochash + ", " + ' '.join(lastmod.split(' ')[:4])

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
try:
    root = ElementTree(None, os.path.join('..', '..', 'package.xml'))
    version = root.findtext('version')
except Exception as e:
    raise RuntimeError('Could not extract version from package.xml:\n%s' % e)

# The full version, including alpha/beta/rc tags.
release = version

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#language = None

# There are two options for replacing |today|: either, you set today to some
# non-false value, then it is used:
#today = ''
# Else, today_fmt is used as the format for a strftime call.
#today_fmt = '%B %d, %Y'

# List of documents that shouldn't be included in the build.
#unused_docs = []

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
exclude_patterns = []

# The reST default role (used for this markup: `text`) to use for all documents.
#default_role = None

# If true, '()' will be appended to :func: etc. cross-reference text.
#add_function_parentheses = True

# If true, the current module name will be prepended to all description
# unit titles (such as .. function::).
#add_module_names = True

# If true, sectionauthor and moduleauthor directives will be shown in the
# output. They are ignored by default.
show_authors = True

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# A list of ignored prefixes for module index sorting.
#modindex_common_prefix = []


# -- Options for HTML output ---------------------------------------------------

# Add any paths that contain custom themes here, relative to this directory.
html_theme_path = [os.path.join(os.path.dirname(catkin_sphinx.__file__),
                                'theme')]

# The theme to use for HTML and HTML Help pages.  Major themes that come with
# Sphinx are currently 'default' and 'sphinxdoc'.
html_theme = 'ros-theme'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
# html_theme_options = { 'rightsidebar' : 'true' }

# The name for this set of Sphinx documents.  If None, it defaults to
# "<project> v<release> documentation".
#html_title = 'catkin'

# A shorter title for the navigation bar.  Default is the same as html_title.
#html_short_title = None

# The name of an image file (relative to this directory) to place at the top
# of the sidebar.
html_logo = 'mrt.png'

# The name of an image file (within the static path) to use as favicon of the
# docs.  This file should be a Windows icon file (.ico) being 16x16 or 32x32
# pixels large.
#html_favicon = None

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
##html_static_path = ['_static']

# If not '', a 'Last updated on:' timestamp is inserted at every page bottom,
# using the given strftime format.
#
# tds: We don't use this, we use the git timestamp
#
# html_last_updated_fmt = '%b %d, %Y'

# If true, SmartyPants will be used to convert quotes and dashes to
# typographically correct entities.
#html_use_smartypants = True

# Custom sidebar templates, maps document names to template names.
#html_sidebars = {}

# Additional templates that should be rendered to pages, maps page names to
# template names.
#html_additional_pages = {}

# If false, no module index is generated.
#html_use_modindex = True

# If false, no index is generated.
#html_use_index = True

# If true, the index is split into individual pages for each letter.
#html_split_index = False

# If true, links to the reST sources are added to the pages.
#html_show_sourcelink = True

# If true, an OpenSearch description file will be output, and all pages will
# contain a <link> tag referring to it.  The value of this option must be the
# base URL from which the finished HTML is served.
#html_use_opensearch = ''

# If nonempty, this is the file name suffix for HTML files (e.g. ".xhtml").
#html_file_suffix = ''

# Output file base name for HTML help builder.
htmlhelp_basename = 'catkin-cmakedoc'


# -- Options for LaTeX output --------------------------------------------------

# The paper size ('letter' or 'a4').
#latex_paper_size = 'letter'

# The font size ('10pt', '11pt' or '12pt').
#latex_font_size = '10pt'

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title, author, documentclass [howto/manual]).
latex_documents = [
  ('generated_cmake_api', 'api.tex', r'CMAKE API',
   r'Fabian Poggenhans', 'manual'),
]

# The name of an image file (relative to this directory) to place at the top of
# the title page.
#latex_logo = None

# For "manual" documents, if this is true, then toplevel headings are parts,
# not chapters.
#latex_use_parts = False

# Additional stuff for the LaTeX preamble.
#latex_preamble = ''

# Documents to append as an appendix to all manuals.
#latex_appendices = []

# If false, no module index is generated.
#latex_use_modindex = True

intersphinx_mapping = {
    'genmsg': ('http://docs.ros.org/indigo/api/genmsg/html', None),
    'vcstools': ('http://docs.ros.org/independent/api/vcstools/html', None),
    'rosinstall': ('http://docs.ros.org/independent/api/rosinstall/html', None),
    'rospkg': ('http://docs.ros.org/independent/api/rospkg/html', None),
    'rosdep': ('http://docs.ros.org/independent/api/rosdep/html', None),
    }


rst_epilog="""


"""
