import copy


class VarSaver:
    def __init__(self, obj):
        """
        obj : L'object dont on va sauvegarder les variables,
        
        Exemple d'utilisation
        self.save = VarSaver(self)
        """
        self.obj            = obj
        self.variables      = {}
        self.var_names      = []
        self.blacklist      = []
        self.whitelist      = []

    def whitelist_add(self, *args):
        for arg in args:
            if type(arg) == list:
                for subarg in arg:
                    if type(subarg) != str:
                        raise TypeError('Variable names must be strings') 
                    self.whitelist.append(subarg)
            elif type(arg) == str:
                self.whitelist.append(arg)
            else:
                raise TypeError('Variable names must be strings') 

    def blacklist_add(self, *args):
        for arg in args:
            if type(arg) == list:
                for subarg in arg:
                    if type(subarg) != str:
                        raise TypeError('Variable names must be strings') 
                    self.blacklist.append(subarg)
            elif type(arg) == str:
                self.blacklist.append(arg)
            else:
                raise TypeError('Variable names must be strings') 


    def list_variables(self):
        self.variables = {}
        for var, value in self.obj.__dict__.items():
            if var in self.blacklist:
                continue

            if self.whitelist == []: # On ajoute tout le monde
                self.variables[var] = copy.copy(value)
                self.var_names.append(var)
            elif var in self.whitelist: # On ajoute que ceux qui sont dans la whitelist
                self.variables[var] = copy.copy(value)
                self.var_names.append(var)
                
    
    def update(self):
        for var in self.var_names:
            self.variables[var] = copy.copy(self.obj.__dict__[var])

    def __getattr__(self, name):
        if name in self.var_names:
            return self.variables[name]
        else:
            raise AttributeError(f'"{name}" not found')