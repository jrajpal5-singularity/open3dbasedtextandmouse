import open3d as o3d
import numpy as np
from PIL import Image, ImageFont, ImageDraw
from pyquaternion import Quaternion
import aspose.threed as a3d

def text_3d(text, pos, direction=None,colorpt=(0,0,0),transparency = False, colorfill=(255, 255, 255),degree=0.0,density=10, font=r'D:\python\keyboard3d\usingopen3d\freemono\FreeMono.ttf', font_size=16):
    """
    Generate a 3D text point cloud used for visualization.
    :param text: content of the text
    :param pos: 3D xyz position of the text upper left corner
    :param direction: 3D normalized direction of where the text faces
    :param degree: in plane rotation of text
    :param font: Name of the font - change it according to your system
    :param font_size: size of the font
    :return: o3d.geoemtry.PointCloud object
    """
    if direction is None:
        direction = (0., 0., 1.)



    font_obj = ImageFont.truetype(font, font_size*density)
    font_dim = font_obj.getsize(text)

    img = Image.new('RGB', font_dim, color=colorfill)
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, font=font_obj, fill=colorpt)
    img = np.asarray(img)
    img_mask = img[:, :, 0] < 256
    indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T

    pcd = o3d.geometry.PointCloud()
    pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float) / 255.0)
    pcd.points = o3d.utility.Vector3dVector(indices / 1000 / density)
    # pcd.points = o3d.utility.Vector3dVector(indices / 100.0)

    raxis = np.cross([0.0, 0.0, 1.0], direction)
    if np.linalg.norm(raxis) < 1e-6:
        raxis = (0.0, 0.0, 1.0)
    trans = (Quaternion(axis=raxis, radians=np.arccos(direction[2])) *
             Quaternion(axis=direction, degrees=degree-90)).transformation_matrix
    trans[0:3, 3] = np.asarray(pos)
    pcd.transform(trans)

    if transparency is True:
        colorptind = np.asarray(pcd.colors)
        ind = np.where(colorptind == colorpt)[0]
        pcdf = pcd.select_by_index(ind,invert=False)
    else:
        pcdf = pcd


    return pcdf

def combinepcwithoutchanging(pc1,pc2):
    p1points = np.asarray(pc1.points)
    p1color = np.asarray(pc1.colors)
    p2points = np.asarray(pc2.points)
    p2color = np.asarray(pc2.colors)
    finalpts = np.concatenate((p1points, p2points), axis=0)
    finalcol = np.concatenate((p1color, p2color), axis=0)
    final = o3d.geometry.PointCloud()

    final.points = o3d.utility.Vector3dVector(finalpts)
    print(finalcol)
    final.colors = o3d.utility.Vector3dVector(finalcol)
    # o3d.visualization.draw_geometries([final])
    return final

def addtextwithshadow(text,pos,posdepth=0.0002,depthdiff=0.00001, transparency=True, direction=(0, 0, 1), colorpt=(0, 0, 255), 
             degree=0.0, colorfill=(255, 255, 255), font_size=5, density=50):
    final = text_3d(text=text, transparency=transparency, direction=direction, colorpt=colorpt, pos=pos, degree=degree,
            colorfill=colorfill, font_size=font_size, density=density)
    location = pos
    for i in np.arange(depthdiff,posdepth,depthdiff):
        print(i)
        location[2] = pos[2] + i
        globals()[f'shadow_{i}'] = text_3d(text=text, transparency=transparency, direction=direction, colorpt=colorpt, pos=location, degree=0.0,
            colorfill=colorfill, font_size=font_size, density=density)
        final = combinepcwithoutchanging(final,globals()[f'shadow_{i}'])
    
    return final
        

if __name__ == "__main__":
    chessboard_coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.02, origin=[0, 0, 0])
    pcd_10 = text_3d('y', pos=[0, 0.01, 0],colorfill= (255,255,255), font_size=10, density=100)
    pcd_20 = text_3d('Test-20mm', pos=[0, 0, 0], font_size=20, density=2)
    # R = (pcd_10.get_rotation_matrix_from_xyz((-0 * np.pi, -0 * np.pi, 1.5* np.pi)))
    # R = pcd_10.get_rotation_matrix_from_quaternion(rotation= np.ndarray[np.float64[3, 3]])
    # R = pcd_10.get_rotation_matrix_from_axis_angle(rotation= np.ndarray[np.float64[3, 3]])

    # print(R)
    # pcd_10_rot = pcd_10.rotate(R,center=(0, 0, 0))
    x = text_3d('x',transparency=True,direction=(0,0,1),colorpt=(255,0,0), pos=[0.015, 0, 0],degree = 0.0, colorfill=(255, 255, 255), font_size=5, density=100)
    y = text_3d('y',transparency=True,direction=(0,0,1),colorpt=(0,255,0), pos=[0.001, 0.015, 0],degree = 0.0, colorfill=(255, 255, 255), font_size=5, density=100)
    z = text_3d('z',transparency=True,direction=(0,0,1),colorpt=(0,0,255), pos=[0, 0.0, 0.015],degree = 0.0, colorfill=(255, 255, 255), font_size=5, density=100)
    # The x, y, z axis will be rendered as red, green, and blue    arrows  respectively.

    # o3d.visualization.draw_geometries([chessboard_coord,x,y,z])
    # pcdmesh = o3d.geometry.PointCloud()
    # pcdmesh.points = chessboard_coord.vertices
    # pcdmesh.colors = chessboard_coord.vertex_colors
    # pcdmesh.normals = chessboard_coord.vertex_normals

    # pcdmesh = chessboard_coord.sample_points_poisson_disk(number_of_points=10000, init_factor=5, pcl=None, use_triangle_normal=False)
    # final = combinepcwithoutchanging(pcdmesh,x)
    # final = combinepcwithoutchanging(final, y)
    # final = combinepcwithoutchanging(final, z)
    # o3d.visualization.draw_geometries([final])
    # o3d.io.write_point_cloud("xyztext.ply",final)
    # scene = a3d.Scene.from_file("xyztext.ply")
    # scene.save("xyztext.gltf")

    hello_world = text_3d('hello world', transparency=True, direction=(0, 0, 1), colorpt=(0, 0, 255), pos=[0, 0.0, 0.015], degree=0.0,
                colorfill=(255, 255, 255), font_size=5, density=500)

    # o3d.visualization.draw_geometries([hello_world])

    yohoo = addtextwithshadow('yohoo', pos=[0, 0.0, 0.015],posdepth=0.0002, transparency=True, direction=(0, 0, 1), colorpt=(0, 0, 255), degree=0.0,
            colorfill=(255, 255, 255), font_size=10, density=50)
    o3d.visualization.draw_geometries([yohoo])
    
