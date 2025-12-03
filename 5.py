import numpy as np
from PIL import Image, ImageOps
from qqq import Quaternion

# Экранные координаты
def zoom(scale, image_height, image_width, x0, y0, z0, x1, y1, z1, x2, y2, z2):  
    nx0 = x0 * scale / z0 + image_width / 2
    ny0 = y0 * scale / z0 + image_height / 2
    nx1 = x1 * scale / z1 + image_width / 2  
    ny1 = y1 * scale / z1 + image_height / 2
    nx2 = x2 * scale / z2 + image_width / 2
    ny2 = y2 * scale / z2 + image_height / 2
    return nx0, ny0, nx1, ny1, nx2, ny2

# Нормаль к плоскости треугольника
def normal(a, b, c): 
    n = np.cross([b[0]-c[0], b[1]-c[1], b[2]-c[2]],
                 [b[0]-a[0], b[1]-a[1], b[2]-a[2]])
    normal_vec = n / np.linalg.norm(n)
    view_dir = [0, 0, 1]
    cos_angle = np.dot(normal_vec, view_dir)
    return cos_angle

# Барицентрические координаты
def barycentric_coords(x, y, x0, y0, x1, y1, x2, y2):
    denominator = (x0 - x2) * (y1 - y2) - (x1 - x2) * (y0 - y2)
    lambda0 = ((x - x2) * (y1 - y2) - (x1 - x2) * (y - y2)) / denominator
    lambda1 = ((x0 - x2) * (y - y2) - (x - x2) * (y0 - y2)) / denominator
    lambda2 = 1.0 - lambda0 - lambda1

    return lambda0, lambda1, lambda2

# Триангуляция многогранников
def polygons_triangulation(str_2, polygons, texture_indices, normals):
    for i in range(2, len(str_2) - 1):
        polygons.append([int(str_2[1].split('/')[0]), int(str_2[i].split('/')[0]), int(str_2[i + 1].split('/')[0])])
        texture_indices.append([int(str_2[1].split('/')[1]), int(str_2[i].split('/')[1]), int(str_2[i + 1].split('/')[1])])
        if len(str_2[1].split('/')) > 2 and str_2[1].split('/')[2]:
            normals.append([int(str_2[1].split('/')[2]), int(str_2[i].split('/')[2]), int(str_2[i + 1].split('/')[2])])

# Парсер
def obj_parser(filename):
    peaks, polygons, texture_coords, texture_indices, normals = [], [], [], [], []

    with open(filename, 'r') as file:
        for line in file:
            str_line = line.strip()
            if str_line == '' or str_line == '\n':
                continue
            str_2 = str_line.split()
            if str_2[0] == 'v':
                peaks.append([float(str_2[1]), float(str_2[2]), float(str_2[3])])

            if str_2[0] == 'vt':
                texture_coords.append([float(str_2[1]), float(str_2[2])])
            
            if str_2[0] == 'f':
                if len(str_2) > 4:
                    polygons_triangulation(str_2, polygons, texture_indices, normals)
                else:
                    polygons.append([int(str_2[1].split('/')[0]), int(str_2[2].split('/')[0]), int(str_2[3].split('/')[0])])
                    texture_indices.append([int(str_2[1].split('/')[1]), int(str_2[2].split('/')[1]), int(str_2[3].split('/')[1])])
                    if len(str_2[1].split('/')) > 2 and str_2[1].split('/')[2]:
                        normals.append([int(str_2[1].split('/')[2]), int(str_2[2].split('/')[2]), int(str_2[3].split('/')[2])])
                    else:
                        normals.append([0, 0, 0])

    return peaks, polygons, texture_coords, texture_indices, normals

# Вычисляем нормали вершин (+ тени Гуро)
def normalize(peaks, polygons):
    normalized_polygons = []

    for i in range(len(polygons)):
        a = peaks[polygons[i][0] - 1]
        b = peaks[polygons[i][1] - 1] 
        c = peaks[polygons[i][2] - 1]
        
        normal_vec = np.cross([b[0]-c[0], b[1]-c[1], b[2]-c[2]], 
                            [b[0]-a[0], b[1]-a[1], b[2]-a[2]])
        norm_val = np.linalg.norm(normal_vec)
        if norm_val > 1e-10:
            normalized_polygons.append(normal_vec / norm_val)
        else:
            normalized_polygons.append([0, 0, 1])

    v_n = np.zeros((len(peaks), 3))
    for i in range(len(polygons)):
        v0 = polygons[i][0] - 1
        v1 = polygons[i][1] - 1
        v2 = polygons[i][2] - 1

        v_n[v0] += normalized_polygons[i]
        v_n[v1] += normalized_polygons[i]
        v_n[v2] += normalized_polygons[i]
    
    for i in range(len(v_n)):
        norm_val = np.linalg.norm(v_n[i])
        if norm_val > 1e-10:
            v_n[i] /= norm_val
        else:
            v_n[i] = [0, 0, 1]

    return v_n

# Повороты и сдвиги модельки
def rotate(peaks, x_angle, y_angle, z_angle, x_shift, y_shift, z_shift):
    a, b, c = np.radians(x_angle), np.radians(y_angle), np.radians(z_angle)
    
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(a), -np.sin(a)],
                   [0, np.sin(a), np.cos(a)]])
    
    Ry = np.array([[np.cos(b), 0, np.sin(b)],
                   [0, 1, 0],
                   [-np.sin(b), 0, np.cos(b)]])
    
    Rz = np.array([[np.cos(c), -np.sin(c), 0],
                   [np.sin(c), np.cos(c), 0],
                   [0, 0, 1]])
    
    R = Rx @ Ry @ Rz

    rotated_peaks = []
    for peak in peaks:
        rotated_peak = R @ np.array(peak) + np.array([x_shift, y_shift, z_shift])
        rotated_peaks.append(rotated_peak.tolist())
    
    return rotated_peaks

# Поворот через кватернион
def quat_rotation(peaks, angle, axis, shift):
    axis_normal = np.array(axis) / np.linalg.norm(axis) # Нормируем ось вращения

    w, xyz = np.cos(angle/2), np.sin(angle / 2) * axis_normal 
    q = Quaternion(w, xyz[0], xyz[1], xyz[2]) 

    q_conj = q.sopryazh() # СОпряженный квартернион
    
    rotated_peaks = []
    for peak in peaks:
        peak_quat = Quaternion(0, peak[0], peak[1], peak[2]) 

         # Применяем вращение: v' = q * v * q^-1
        peak_rotated = q * peak_quat * q_conj
        rotated_peak = [peak_rotated.x + shift[0], peak_rotated.y + shift[1], peak_rotated.z + shift[2]] 
        rotated_peaks.append(rotated_peak)
    return rotated_peaks


# Отрисовка треугольника
def draw_triangle(scale, image_height, image_width, texture_width, texture_height, 
                 x0, y0, z0, x1, y1, z1, x2, y2, z2, 
                 tx0, ty0, tx1, ty1, tx2, ty2, texture, I0, I1, I2, img, z_buff):
    
    nx0, ny0, nx1, ny1, nx2, ny2 = zoom(scale, image_height, image_width, x0, y0, z0, x1, y1, z1, x2, y2, z2)
    
    xmin = min(int(nx0), int(nx1), int(nx2))
    if xmin < 0: xmin = 0
    xmax = max(int(nx0), int(nx1), int(nx2))
    if xmax >= image_width: xmax = image_width - 1

    ymin = min(int(ny0), int(ny1), int(ny2))
    if ymin < 0: ymin = 0
    ymax = max(int(ny0), int(ny1), int(ny2)) 
    if ymax >= image_height: ymax = image_height - 1

    cos_angle = normal([x0, y0, z0], [x1, y1, z1], [x2, y2, z2])

    if cos_angle > 0:
        return
    
    for x in range(xmin, xmax + 1):
        for y in range(ymin, ymax + 1):
            lambda0, lambda1, lambda2 = barycentric_coords(x, y, nx0, ny0, nx1, ny1, nx2, ny2)

            if lambda0 >= 0 and lambda1 >= 0 and lambda2 >= 0:
                z = z0 * lambda0 + z1 * lambda1 + z2 * lambda2 
                I = (lambda0 * I0 + lambda1 * I1 + lambda2 * I2)
                    
                if z < z_buff[y, x] and z > 0:
                    u = lambda0 * tx0 + lambda1 * tx1 + lambda2 * tx2
                    v = lambda0 * ty0 + lambda1 * ty1 + lambda2 * ty2

                    texture_x_coord = int(u * (texture_width - 1))
                    texture_y_coord = int((1 - v) * (texture_height - 1))

                    texture_x_coord = max(0, min(texture_width - 1, texture_x_coord))
                    texture_y_coord = max(0, min(texture_height - 1, texture_y_coord))
                    
                    texture_color = texture.getpixel((texture_x_coord, texture_y_coord))
                    intensity = max(0.1, min(1.0, abs(I)))
                    final_texture_color = (intensity * np.array(texture_color)).astype(np.uint8)
                    img[y, x] = final_texture_color
                    z_buff[y, x] = z

# Отрисовка модели
def draw_model(peaks, polygons, texture_coords, texture_indices, texture, scale, image_height, image_width, img, z_buff):
    v_n = normalize(peaks, polygons)
    texture_width, texture_height = texture.size
    
    for i in range(len(polygons)):
                
        x0, y0, z0 = peaks[polygons[i][0] - 1]
        x1, y1, z1 = peaks[polygons[i][1] - 1]
        x2, y2, z2 = peaks[polygons[i][2] - 1]
            
        tx0, ty0 = texture_coords[texture_indices[i][0] - 1]
        tx1, ty1 = texture_coords[texture_indices[i][1] - 1]  
        tx2, ty2 = texture_coords[texture_indices[i][2] - 1]

        I0 = np.dot(v_n[polygons[i][0] - 1], [0, 0, 1])
        I1 = np.dot(v_n[polygons[i][1] - 1], [0, 0, 1])
        I2 = np.dot(v_n[polygons[i][2] - 1], [0, 0, 1])
        
        draw_triangle(scale, image_height, image_width, texture_width, texture_height,
                     x0, y0, z0, x1, y1, z1, x2, y2, z2,
                     tx0, ty0, tx1, ty1, tx2, ty2, texture, 
                     I0, I1, I2, img, z_buff)


# Scale - 2000, Z - 0.2, 2000 * 2000
filename1 = 'model_1.obj'   
texture1 = Image.open('bunny-atlas.jpg') 

# Scale - 2000, Z - 15, 2000 * 2000
filename2 = 'frog.obj'
texture2 = Image.open('frog.jpg')

image_height, image_width = 300, 600
scale = 2000

img = np.zeros((image_height, image_width, 3), dtype=np.uint8)

peaks1, polygons1, texture_coords1, texture_indices1, normals1 = obj_parser(filename1)
peaks2, polygons2, texture_coords2, texture_indices2, normals2 = obj_parser(filename2)

choice = int(input("Выбери способ поворота: 1 - matrix_rotate, 2 - quaternion_rotate: "))

if choice == 1:
    peaks1 = rotate(peaks1, 0, 180, 0, -0.07, 0, 0.2)
    peaks2 = rotate(peaks2, 135, 15, 0, 0, 0, 14)
else:
    peaks1 = quat_rotation(peaks1, np.radians(75), [1,1,1], [0.01, 0, 0.5])
    peaks2 = quat_rotation(peaks2, np.radians(75), [1, 1, 1], [0, 0, 14])

z_buff = np.full((image_height, image_width), np.inf, dtype=np.float32)

draw_model(peaks1, polygons1, texture_coords1, texture_indices1, texture1, scale, image_height, image_width, img, z_buff)
print('Первая модель отрисована')
draw_model(peaks2, polygons2, texture_coords2, texture_indices2, texture2, scale, image_height, image_width, img, z_buff)

image = Image.fromarray(img, mode='RGB')
image = ImageOps.flip(image)
image.save('quaternion_rotate_bunny_and_frog_3.png')