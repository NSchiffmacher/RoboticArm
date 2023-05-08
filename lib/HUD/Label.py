import pygame
from pygame.locals import *

from lib.Math.Vector import Vector2

from lib.HUD.CanvasItem import CanvasItem


class Label(CanvasItem):
    TEXT_LEFT       = 0
    TEXT_CENTERED   = 1
    TEXT_RIGHT      = 2

    TAB_LENGTH      = 3
    NO_PRINT_CHARS  = ['\t', '\n']

    def __init__(self, parent: pygame.Surface, font: pygame.font.Font or None = None, text: str = "", text_align: int = TEXT_LEFT, color: list or tuple or None = None, position: Vector2 or None = None, size: Vector2 or None = None):
        self.init_canvas(parent)

        self.text           = text
        self.font           = font
        self.lines          = None

        self.line_offset    = 0
        self.text_align     = text_align
        self.yscroll_offset = 0

        self.word_wrapping = True
        
        if font:
            self.set_font(font)
        if color:
            self.set_color(color)
        else:
            self.color = None
        if position:
            self.set_position(position)
        if size:
            self.set_size(size)


    def load_font(self, font_file, font_size):
        self.font           = pygame.font.Font(font_file, font_size)
        self.line_height    = self.font.get_linesize()
    
    def set_font(self, font):
        self.font = font
        self.line_height = font.get_linesize()

    def set_yscroll_offset(self, y):
        self.yscroll_offset = y
        return self


    def change_text(self, text):
        self.text = text
        self.load()
        return self

    def load(self):
        if self.word_wrapping:
            self.lines = []
            words = self.text.split(' ')
            sentence = words[0]
            for word in words[1:]:
                if word == '\t':
                    word = '    '

                if self.font.size(sentence + " " + word)[0] <= self.size.x and word != '\n':
                    sentence += ' ' + word
                else:
                    self.add_line(sentence)
                    sentence = word
            
            if sentence.strip() != '':
                self.add_line(sentence)
        
        else:
            self.lines = []
            self.add_line(self.text)
                
        return self
    def add_line(self, text):
        for forbidden_char in self.NO_PRINT_CHARS:
            text = text.replace(forbidden_char, '')

        surface = self.font.render(text, True, self.color)
        width = surface.get_width()
        xoffset = 0
        if self.text_align == self.TEXT_CENTERED:
            xoffset = self.size.x / 2 - width / 2
        elif self.text_align == self.TEXT_RIGHT:
            xoffset = self.size.x - width

        self.lines.append({
            "surface": surface,
            "xoffset": xoffset,
            "width"  : width
        })


    def set_text_align(self, code: int):
        if 0 <= code <= 2:
            self.text_align = code
        else:
            raise AttributeError('Text align code invalid (must be in [|0;2|])')
        return self
        

    def get_display_height(self):
        L = len(self.lines)
        return self.line_height * L + self.line_offset * (L-1)

    def fit_height(self):
        if self.size == None:
            self.size = Vector2()

        if self.lines == None:
            raise SystemError('Label not loaded')

        L = len(self.lines)
        self.size.y = self.line_height * L + self.line_offset * (L-1)
        return self
        
    def set_line_offset(self, line_offset: int):
        self.line_offset = int(line_offset)
        return self

    def set_color(self, color: list or tuple):
        if len(color) != 3:
            raise AttributeError(f'Colors must be RGB')

        for color_val in color:
            if not 0 <= color_val <= 255:
                raise AttributeError('Colors are RGB, from 0 to 255')
        
        self.color = color
        return self

    def draw(self):
        if self.position == None:
            raise AttributeError('Position not set')
        if self.size == None:
            raise AttributeError('Size not set')
        if self.color == None:
            raise AttributeError('Color not set')
        if self.lines == None:
            raise SystemError('Label not loaded')

        for i, line in enumerate(self.lines):
            offset = Vector2(line['xoffset'], i * (self.line_height + self.line_offset)) - Vector2(0,self.yscroll_offset)
            if offset.y + self.line_height > self.size.y or offset.y < 0:
                continue
            self.parent.surface.blit(line['surface'], (self.position + offset).to_pygame())

        return self
        # pygame.draw.rect(self.parent.surface, self.color, (self.position.x, self.position.y, self.size.x, self.size.y))