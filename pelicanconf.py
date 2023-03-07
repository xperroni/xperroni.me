AUTHOR = 'Helio Perroni Filho'
SITENAME = 'Home Page of Helio Perroni Filho'
SITEURL = ''

PATH = 'content'

TIMEZONE = 'America/Toronto'
LOCALE = 'en_CA.utf8'
DEFAULT_LANG = 'en'

# Feed generation is usually not desired when developing
FEED_ALL_ATOM = None
CATEGORY_FEED_ATOM = None
TRANSLATION_FEED_ATOM = None
AUTHOR_FEED_ATOM = None
AUTHOR_FEED_RSS = None

THEME = '../pelican-themes/gum'

INDEX_SAVE_AS = 'posts.html'
MENUITEMS = (
    ('Posts', '/posts.html'),
)

# See: https://github.com/getpelican/pelican-plugins/pull/1094
MATH_JAX = {
    'equation_numbering': 'AMS'
}

# Blogroll
#LINKS = (('Pelican', 'https://getpelican.com/'),
         #('Python.org', 'https://www.python.org/'),
         #('Jinja2', 'https://palletsprojects.com/p/jinja/'),
         #('You can modify those links in your config file', '#'),)

# Social widget
#SOCIAL = (('You can add links in your config file', '#'),
          #('Another social link', '#'),)

DEFAULT_PAGINATION = False

# Uncomment following line if you want document-relative URLs when developing
#RELATIVE_URLS = True
