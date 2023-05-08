import os, sys
import json

from lib.App import App

os.environ['SDL_VIDEO_CENTERED'] = '1'
with open('options.json') as file:
    options = json.loads(file.read())

app = App(options)
app.load()
app.run()