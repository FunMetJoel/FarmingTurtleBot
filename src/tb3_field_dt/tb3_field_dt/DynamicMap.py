import numpy as np

class DynamicMap():
    """A map that can dynamically grow
    
    Parameters:  
    resolution (float): meter per pixel
    """
    def __init__(self, resolution: float, origin:tuple[float, float] = (0.0, 0.0), startSize:tuple[float, float] = (10, 10)):
        self.resolution:float = resolution
        self.origin = origin
        self.size = startSize
        self.array:np._ArrayFloat_co = np.full(startSize, np.nan, dtype=np.int8)
    
    def setPixelAtLocation(self, worldX:float, worldY:float, value:float):
        pX, pY = self.worldToPixelCords(worldX, worldY)
        if not self.__inBounds(pX, pY):
            self.__expandToInclude(self, pX, pY)
            pX, pY = self.worldToPixelCords(worldX, worldY)
        self.__setPixel(pX, pY, value)
    
    def __setPixel(self, x:int, y:int, value:float):
        self.array[x, y] = value

    def __expandToInclude(self, x:int, y:int):
        if (self.size[0] < x):
            self.size[0] = x + 1
        elif (x < 0):
            self.origin[0] = float(x) * self.resolution + self.origin[0]
            self.size[0] = self.size[0] - x

        if (self.size[1] < y):
            self.size[1] = y + 1
        elif (y < 0):
            self.origin[1] = float(x) * self.resolution + self.origin[1]
            self.size[1] = self.size[1] - x

    def __inBounds(self, x:int, y:int) -> bool:
        return (self.size[0] > x) and (0 < x) and (self.size[1] > y) and (0 < y)
    
    def pixelToWorldCords(self, x:int, y:int) -> tuple[float, float]: 
        worldX:float = (float(x) * self.resolution) + self.origin[0]
        worldY:float = (float(y) * self.resolution) + self.origin[1]
        return worldX, worldY
    
    def worldToPixelCords(self, x:float, y:float) -> tuple[int, int]:
        pixelX:int = round((x - self.origin[0]) / self.resolution)
        pixelY:int = round((y - self.origin[1]) / self.resolution)
        return pixelX, pixelY


