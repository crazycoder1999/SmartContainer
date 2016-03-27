from flask import Flask
from flask.ext import shelve

app = Flask(__name__);
from app import views
app.config['SHELVE_FILENAME'] = 'shelve.db'
shelve.init_app(app)
