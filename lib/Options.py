class Container:
    def __init__(self, name = 'Null'):
        self.__name__ = name

    def names(self):
        res = []
        for attr in self.__dict__:
            if not attr.startswith('__'):
                res.append(attr)
        return res
    
    def values(self):
        res = []
        for attr in self.__dict__:
            if not attr.startswith('__'):
                res.append(self.__dict__[attr])
        return res
    
    def items(self):
        res = []
        for attr in self.__dict__:
            if not attr.startswith('__'):
                res.append([attr, self.__dict__[attr]])
        return res

    def __contains__(self, key):
        return key in self.__dict__

    def print(self, depth = 0, indent = 3):
        headline = f'│{" " * indent}' * depth
        headline += f'├{"─" * indent}' + self.__name__
        print(headline)


        for attr in self.names():
            if type(self.__dict__[attr]) == Container:
                self.__dict__[attr].print(depth + 1)
            else:
                headline = f'│{" " * indent}' * (depth + 1)
                headline += f'├{"─" * indent}' + str(type(self.__dict__[attr])) + ' ' + attr + ': ' + str(self.__dict__[attr])
                print(headline)
        

class Options():    
    def __init__(self, json):
        self.load(self, json)
    
    def print(self, depth = 0, indent = 6):
        headline = f'│{" " * indent}' * depth
        headline += f'│{"─" * indent}' + 'Options'
        print(headline)

        for attr in self.__dict__:
            if type(self.__dict__[attr]) == Container:
                self.__dict__[attr].print(attr, depth + 1, indent)
            else:
                headline = f'│{" " * indent}' * (depth + 1)
                headline += f'│{"─" * indent}' + str(type(self.__dict__[attr])) + ' ' + attr + ': ' + str(self.__dict__[attr])
                print(headline)

            

    def load(self, obj, D : dict):
        for key in D:
            if type(D[key]) == dict:
                O = Container(key) # new object
                self.load(O, D[key])
                setattr(obj, key, O)
            else:
                setattr(obj, key, D[key])