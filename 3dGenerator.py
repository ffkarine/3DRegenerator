import os
import math
import numpy as np
import pandas as pd
import open3d as o3d
import copy
from typing import List

class MeshFromCSV:
    __degreeToRad : float
    __stepToDegree : float
    __stepToRad : float
    __totalVertexCounter : int
    __yCounter : int

    stepsTotal : int
    yIncrement : float
    Mesh : o3d.geometry.TriangleMesh

    __tv1 = []
    __tv2 = []
    __tv3 = []

    __vx = []
    __vy = []
    __vz = []


    def __init__(self, stepsTotal: int = 4000, yIncrement: float = 10.0, CSVsFolder : str = "/csvs/"):
        self.__totalVertexCounter = 0
        self.__degreeToRad = math.pi/180
        self.__stepsTotal = stepsTotal
        self.__stepToDegree = 360/stepsTotal
        self.__stepToRad = self.__stepToDegree * self.__degreeToRad
        self.__yCounter = 0
        self.yIncrement = yIncrement

        # # sets the working directory to the same as the current file
        file_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(file_path + CSVsFolder)

    def TriangulatePoints(self):
        self.__yCounter = 0
        validFileCounter = 0
        vfc = 0

        # Counts the valid files in the given directory (.csv files)
        for file in os.listdir():
            if(file[ -3 : ] != 'csv'):
                continue
            validFileCounter += 1

        for file in os.listdir():
            if(file[ -3 : ] != 'csv'):
                continue

            vfc += 1
            csv = pd.read_csv(file, sep=';')
            csv = csv.copy()

            distances = csv['distancia'].astype('float32')
            distances = distances.values
            steps = csv['angulo'].astype('int32')
            steps = steps.values

            # # The first valid file will be used to generate the first face/plane
            if(vfc == 1):
                self.__CreateFrontPlane(steps, distances)

            # # The last valid file will be used to generate the back face/plane and close the solid
            elif(vfc == validFileCounter):
                self.__CreateClosedBlackPlane(steps, distances)

            # # Any intermediate files will be used as middle slices/layers, closing the side of the solid with the previous layer 
            else:
                self.__CreateSidesFromMiddleSlices(steps, distances)

            self.__yCounter += self.yIncrement

        # # Generates the mesh with the data obtained (vertex/points read from the files and the triangles generated)
        self.__vx = np.array(self.__vx)
        self.__vy = np.array(self.__vy)
        self.__vz = np.array(self.__vz)

        self.__tv1 = np.array(self.__tv1)
        self.__tv2 = np.array(self.__tv2)
        self.__tv3 = np.array(self.__tv3)

        vertices = np.array([self.__vx, self.__vy, self.__vz]).T
        triangles = np.array([self.__tv3, self.__tv2, self.__tv1]).T.astype(np.int32)

        mesh = o3d.geometry.TriangleMesh( vertices = o3d.utility.Vector3dVector(vertices), triangles = o3d.utility.Vector3iVector(triangles) )
        mesh.compute_triangle_normals()
        mesh.compute_vertex_normals()
        self.MeshObject = mesh

        return copy.deepcopy(mesh)

    def __CreateFrontPlane(self, steps : np.ndarray, distances : np.ndarray ):
        i = 1
        self.__totalVertexCounter += 1

        while i < len(distances):
            # # Θ : rad
            # # Xf = distance * cos( Θ )
            # # Zf = distance * sin( Θ )

            self.__totalVertexCounter += 1

            # # New triangle in the front plane formed by the latest points and the origin
            self.__tv3.append(0)
            self.__tv2.append(i)
            self.__tv1.append(i+1)

            if(i == 1):
                # # origin of the current layer/slice
                self.__vx.append( 0 )
                self.__vy.append( 0 )
                self.__vz.append( 0 )

                # # Previous vertex
                x = distances[i - 1] * math.cos( (steps[i - 1]) * self.__stepToRad )
                z = distances[i - 1] * math.sin( (steps[i - 1]) * self.__stepToRad )

                self.__vx.append( x )
                self.__vy.append(self.__yCounter)
                self.__vz.append( z )
            
                # # Current vertex
                x = distances[i] * math.cos( (steps[i]) * self.__stepToRad )
                z = distances[i] * math.sin( (steps[i]) * self.__stepToRad )

                self.__vx.append( x )
                self.__vy.append(self.__yCounter)
                self.__vz.append( z )

                i += 1
                
                continue

            # # Current vertex
            x = distances[i] * math.cos( (steps[i]) * self.__stepToRad )
            z = distances[i] * math.sin( (steps[i]) * self.__stepToRad )

            self.__vx.append( x )
            self.__vy.append( self.__yCounter )
            self.__vz.append( z )
            
            i += 1

        # # closes the front plane with the origin, the last vertex and the first vertex
        self.__tv3.append(0)
        self.__tv2.append(i)
        self.__tv1.append(1)

    def __CreateClosedBlackPlane(self, steps : np.ndarray, distances : np.ndarray):
        i = 1
        self.__totalVertexCounter += 1

        while i < len(distances):
            # # Θ : rad
            # # Xf = distance * cos( Θ )
            # # Zf = distance * sin( Θ )

            self.__totalVertexCounter += 1

            # # New triangle in the back plane formed by the latest points and the origin of the current layer/slice
            self.__tv1.append(int((self.__totalVertexCounter - 1)/len(distances)) * len(distances) + 1)
            self.__tv2.append(self.__totalVertexCounter)
            self.__tv3.append(self.__totalVertexCounter + 1)
                
            if(i == 1):
                # # origin of the current layer/slice
                self.__vx.append( 0 )
                self.__vy.append( self.__yCounter )
                self.__vz.append( 0 )

                # # Previous vertex
                x = distances[i - 1] * math.cos( (steps[i - 1]) * self.__stepToRad )
                z = distances[i - 1] * math.sin( (steps[i - 1]) * self.__stepToRad )

                self.__vx.append( x )
                self.__vy.append( self.__yCounter )
                self.__vz.append( z )
            
                # # Current vertex
                x = distances[i] * math.cos( (steps[i]) * self.__stepToRad )
                z = distances[i] * math.sin( (steps[i]) * self.__stepToRad )

                self.__vx.append( x )
                self.__vy.append( self.__yCounter )
                self.__vz.append( z )

                i += 1

                # # closes the side with the first two points from the current layer 
                # # and the equivalent first point in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter )
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + 1)
                self.__tv3.append( self.__totalVertexCounter + 1 )

                # # closes the side with the second point from the current layer 
                # # and the equivalent first two points in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter + 1 )
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + 1)
                self.__tv3.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + 2)                    
                
                continue

            if( i != len(distances) - 1):
                # # closes the side with the latest two points from the current layer 
                # # and the equivalent "first of the latest" point in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter )
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + i)
                self.__tv3.append( self.__totalVertexCounter + 1)

                # # closes the side with the latest point from the current layer 
                # # and the equivalent two "latest" points in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter + 1 )
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + i)
                self.__tv3.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + i + 1)


            else:
                # # closes the side with the latest two points from the current layer 
                # # and the equivalent "first of the latest" point in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter)
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i)
                self.__tv3.append( self.__totalVertexCounter + 1)

                # # closes the side with the latest point from the current layer 
                # # and the equivalent two "latest" points in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter + 1)
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i)
                self.__tv3.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i + 1)

                # # closes the side with the latest point from the current layer 
                # # and the equivalent two "latest" points in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter + 1)
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i + 1)
                self.__tv3.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i + 3)

            # # Current vertex
            x = distances[i] * math.cos( (steps[i]) * self.__stepToRad )
            z = distances[i] * math.sin( (steps[i]) * self.__stepToRad )

            self.__vx.append( x )
            self.__vy.append( self.__yCounter )
            self.__vz.append( z )
            
            i += 1

        # # closes last side triangle
        self.__tv1.append(int((self.__totalVertexCounter - 1)/len(distances)) * len(distances) + 1)
        self.__tv2.append(self.__totalVertexCounter + 1)
        self.__tv3.append(int((self.__totalVertexCounter - 1)/len(distances)) * len(distances) + 2)
        
        # # closes last back plane triangle
        self.__tv1.append( (int(self.__totalVertexCounter/len(distances)) - 1) * len(distances) + 2 )
        self.__tv2.append( (int(self.__totalVertexCounter/len(distances)) - 1) * len(distances))
        self.__tv3.append( (int(self.__totalVertexCounter/len(distances)) - 1) * len(distances) - len(distances) + 1 )

    def __CreateSidesFromMiddleSlices(self, steps : np.ndarray, distances : np.ndarray):
        i = 1
        self.__totalVertexCounter += 1

        while i < len(distances):
            # # Θ : rad
            # # Xf = distance * cos( Θ )
            # # Zf = distance * sin( Θ )

            self.__totalVertexCounter += 1

            if(i == 1):
                # # Previous vertex
                x = distances[i - 1] * math.cos( (steps[i - 1]) * self.__stepToRad )
                z = distances[i - 1] * math.sin( (steps[i - 1]) * self.__stepToRad )

                self.__vx.append( x )
                self.__vy.append( self.__yCounter )
                self.__vz.append( z )
            
                # # Current vertex
                x = distances[i] * math.cos( (steps[i]) * self.__stepToRad )
                z = distances[i] * math.sin( (steps[i]) * self.__stepToRad )

                self.__vx.append( x )
                self.__vy.append( self.__yCounter )
                self.__vz.append( z )

                i += 1

                # # closes the side with the first two points from the current layer 
                # # and the equivalent first point in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter - 1)
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + i - 1)
                self.__tv3.append( self.__totalVertexCounter )

                # # closes the side with the second point from the current layer 
                # # and the equivalent first two points in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter )
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + i - 1)
                self.__tv3.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + 2 )
                
                continue

            if( i != len(distances) - 1):
                # # closes the side with the latest two points from the current layer 
                # # and the equivalent "first of the latest" point in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter - 1)
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + i)
                self.__tv3.append( self.__totalVertexCounter )

               # # closes the side with the latest point from the current layer 
                # # and the equivalent two "latest" points in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter )
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + i)
                self.__tv3.append( int((self.__totalVertexCounter)/len(distances)) * len(distances) - len(distances) + i + 1)


            else:
                # # closes the side with the latest two points from the current layer 
                # # and the equivalent "first of the latest" point in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter - 1)
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i)
                self.__tv3.append( self.__totalVertexCounter )

                # # closes the side with the latest point from the current layer 
                # # and the equivalent two "latest" points in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter )
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i)
                self.__tv3.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i + 1)

                # # closes the side with the latest point from the current layer 
                # # and the equivalent two "latest" points in the previous layer/slice
                self.__tv1.append( self.__totalVertexCounter )
                self.__tv2.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i + 1)
                self.__tv3.append( int((self.__totalVertexCounter)/len(distances) - 1) * len(distances) - len(distances) + i + 2)

            # # Current vertex
            x = distances[i] * math.cos( (steps[i]) * self.__stepToRad )
            z = distances[i] * math.sin( (steps[i]) * self.__stepToRad )

            self.__vx.append( x )
            self.__vy.append( self.__yCounter )
            self.__vz.append( z )
            
            i += 1
        
        # # closes last side triangle
        self.__tv1.append( (int(self.__totalVertexCounter/len(distances)) - 2) * len(distances) + i + 1)
        self.__tv2.append( (int(self.__totalVertexCounter/len(distances)) - 1) * len(distances) )
        self.__tv3.append( (int(self.__totalVertexCounter/len(distances)) - 1) * len(distances) - len(distances) + 1 )


# # Creates the xyx orientation axes -- normalized if no lenght parameters are provided
def CreateCoordinateAxis(xMax : int = 1, yMax : int = 1, zMax : int = 1):
    points = [[0, 0, 0], [xMax, 0, 0], [0, yMax, 0], [0, 0, zMax], [-xMax, 0, 0], [0, -yMax, 0], [0, 0, -zMax]]
    lines = [[0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [.5, 0, 0], [0, .5, 0], [0, 0, .5]]

    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(points)
    axis.lines = o3d.utility.Vector2iVector(lines)
    axis.colors = o3d.utility.Vector3dVector(colors)

    return axis


# # Performs a boolean difference between two TriangleMeshes. 
# # A new color is computed for the resulting object based on the difference between the colors of the original objects.
# # The resulting object is returned.
def BooleanDifference(minuend : o3d.geometry.TriangleMesh, subtrahend : o3d.geometry.TriangleMesh ):
    minu = copy.deepcopy( o3d.t.geometry.TriangleMesh.from_legacy( minuend ) )
    subtra = copy.deepcopy( o3d.t.geometry.TriangleMesh.from_legacy( subtrahend ) )

    result = minu.boolean_difference(subtra, tolerance= 1e-03)
    result = o3d.t.geometry.TriangleMesh.to_legacy(result)
    result.compute_triangle_normals()
    result.compute_vertex_normals()

    R = np.mod( np.add( np.asarray( minuend.vertex_colors )[1, 0], -np.asarray( subtrahend.vertex_colors )[1, 0] ) * 255, 256 ) / 256
    G = np.mod( np.add( np.asarray( minuend.vertex_colors )[1, 1], -np.asarray( subtrahend.vertex_colors )[1, 1] ) * 255, 256 ) / 256
    B = np.mod( np.add( np.asarray( minuend.vertex_colors )[1, 2], -np.asarray( subtrahend.vertex_colors )[1, 2] ) * 255, 256 ) / 256

    result.paint_uniform_color( np.asarray([[R], [G], [B]]) ) 

    return result

# # Performs a boolean intersection between two TriangleMeshes. 
# # A new color is computed for the resulting object based on the product between the colors of the original objects.
# # The resulting object is returned.
def BooleanIntersection(meshA : o3d.geometry.TriangleMesh, meshB : o3d.geometry.TriangleMesh ):
    ma = copy.deepcopy( o3d.t.geometry.TriangleMesh.from_legacy( meshA ) )
    mb = copy.deepcopy( o3d.t.geometry.TriangleMesh.from_legacy( meshB ) )

    result = ma.boolean_intersection(mb, tolerance= 1e-03)
    result = o3d.t.geometry.TriangleMesh.to_legacy(result)
    result.compute_triangle_normals()
    result.compute_vertex_normals()

    R = np.mod( ( np.asarray( meshA.vertex_colors )[1, 0] * np.asarray( meshB.vertex_colors )[1, 0] ) * 255, 256 ) / 256
    G = np.mod( ( np.asarray( meshA.vertex_colors )[1, 1] * np.asarray( meshB.vertex_colors )[1, 1] ) * 255, 256 ) / 256
    B = np.mod( ( np.asarray( meshA.vertex_colors )[1, 2] * np.asarray( meshB.vertex_colors )[1, 2] ) * 255, 256 ) / 256

    result.paint_uniform_color( np.asarray([[R], [G], [B]]) ) 

    return result

# # Performs a boolean union between two TriangleMeshes. 
# # A new color is computed for the resulting object based on the addition between the colors of the original objects.
# # The resulting object is returned.
def BooleanUnion(add1 : o3d.geometry.TriangleMesh, add2 : o3d.geometry.TriangleMesh ):
    a = copy.deepcopy( o3d.t.geometry.TriangleMesh.from_legacy( add1 ) )
    b = copy.deepcopy( o3d.t.geometry.TriangleMesh.from_legacy( add2 ) )

    result = a.boolean_union(b, tolerance= 1e-03)
    result = o3d.t.geometry.TriangleMesh.to_legacy(result)
    result.compute_triangle_normals()
    result.compute_vertex_normals()

    R = np.mod( np.add( np.asarray( add1.vertex_colors )[1, 0], np.asarray( add2.vertex_colors )[1, 0] ) * 255, 256 ) / 256
    G = np.mod( np.add( np.asarray( add1.vertex_colors )[1, 1], np.asarray( add2.vertex_colors )[1, 1] ) * 255, 256 ) / 256
    B = np.mod( np.add( np.asarray( add1.vertex_colors )[1, 2], np.asarray( add2.vertex_colors )[1, 2] ) * 255, 256 ) / 256

    result.paint_uniform_color( np.asarray([[R], [G], [B]]) ) 

    return result

def main():
    # # Generate a Mesh from the CSVs
    meshfromcsv = MeshFromCSV(stepsTotal= 4000, yIncrement= 10, CSVsFolder= "/csvs/")
    mesh = meshfromcsv.TriangulatePoints()

    # # Translate the generated mesh to the origin
    mesh.translate(-mesh.get_center())

    # # Rotates the mesh in 270deg
    mesh.rotate(mesh.get_rotation_matrix_from_axis_angle([np.radians(-90), 0, 0]))

    # # Paints the mesh dark yellow
    mesh.paint_uniform_color( np.array([ [0.6], [0.4], [0.0] ])  )


    # # Creates a copy of the generated mesh...
    mesh2 = copy.deepcopy(mesh)

    # # The .2deg rotation below is applied because Open3D boolean operations are based on VTK, which doesn't support operations between coplanar meshes
    # # This doesn't solve the problem, but allows the operations to be performed on the meshes in a approximated way
    mesh2.rotate(mesh.get_rotation_matrix_from_axis_angle([0, np.radians(180), np.radians(.2)]))
    mesh2.compute_triangle_normals()
    mesh2.compute_vertex_normals()

    vx = np.asarray(mesh2.vertices)[:, 0] 
    vy = np.asarray(mesh2.vertices)[:, 1] 
    vz = np.asarray(mesh2.vertices)[:, 2]

    # # ... and translates it in the x-axis
    vx = [ v + 75 for v in vx]
    # # vy = [ v for v in vy]
    # # vz = [ v for v in vz]
    mesh2.vertices = o3d.cpu.pybind.utility.Vector3dVector( np.array([vx, vy, vz]).T )
    
    # # paints the copied mesh purple
    mesh2.paint_uniform_color( np.array([ [0.4], [0.1], [0.6] ]) )

    # result = BooleanDifference(mesh, mesh2)
    # result = BooleanUnion(mesh, mesh2)
    result = BooleanIntersection(mesh, mesh2)

    # # Saves the resulting mesh as a .stl file
    o3d.io.write_triangle_mesh( os.path.dirname(os.path.abspath(__file__)) + "\\mesh.stl", result, write_ascii=False)

    # # Generates the reference xyz axis from the origin
    axis = CreateCoordinateAxis(xMax= 100, yMax= 100, zMax= 100)

    # # o3d.visualization.draw_geometries([axis, mesh, mesh2], mesh_show_wireframe = False)
    o3d.visualization.draw_geometries([axis, result], mesh_show_wireframe = False)

if __name__ == '__main__':
    main()