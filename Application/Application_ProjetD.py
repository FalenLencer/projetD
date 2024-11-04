import sys
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel, QFileDialog, QVBoxLayout, 
                             QWidget, QMessageBox, QSlider, QListWidget, QHBoxLayout, QMenuBar, QAction, QDialog)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon, QPixmap

import numpy as np
from math import pi
import open3d as o3d

def rad(x):
    return (x * pi) / 180

def conversion(points, rcut=100000):
    r, t, p = points
    if r > rcut: 
        return None
    x = r * np.sin(rad(t)) * np.cos(rad(p))
    y = r * np.sin(rad(t)) * np.sin(rad(p))
    z = r * np.cos(rad(t))
    return x, y, z

def traitement(points, range_value):
    x, y, z = points
    if (float(x) < -range_value or float(x) > range_value or 
        float(y) < -range_value or float(y) > range_value or 
        float(z) < -range_value or float(z) > range_value or 
        float(x) == 0 or float(y) == 0 or float(z) == 0):
        return True
    return False

def load_point_cloud(file_path):
    try:
        pcd = o3d.io.read_point_cloud(file_path)
        if pcd.is_empty():
            raise ValueError("Le nuage de points est vide ou invalide.")
        return pcd
    except Exception as e:
        QMessageBox.critical(None, "Erreur", f"Impossible de charger le nuage de points : {str(e)}")
        return None

class CreditsDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Crédits")
        self.setGeometry(400, 400, 300, 200)

        layout = QVBoxLayout()

        # QLabel pour afficher l'image
        self.image_label = QLabel()
        self.image_pixmap = QPixmap("MainCaracters.png")
        self.image_label.setPixmap(self.image_pixmap)
        self.image_label.setScaledContents(True)
        self.image_label.setFixedSize(600, 400)

        layout.addWidget(self.image_label)

        credits_text = QLabel("Projet D : Ce projet a été réalisé par Aldric Duwelz et Clément Gaschet.\nIl a pour objectif d'être capable de scanner une pièce et d'en obtenir un fichier 3D (STL)")  # Remplacez par le nom du créateur
        credits_text.setStyleSheet("font-size: 20px;")
        credits_text.setAlignment(Qt.AlignCenter)
        layout.addWidget(credits_text)

        self.setLayout(layout)

class PointCloudApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowIcon(QIcon("Cannard.png"))
        self.setWindowTitle("Point Cloud Converter")
        self.setGeometry(300, 300, 400, 600)

        # Layout principal
        main_layout = QHBoxLayout()

        # Layout pour les boutons et les labels
        layout = QVBoxLayout()

        self.label = QLabel("Sélectionnez un fichier CSV à convertir")
        layout.addWidget(self.label)

        self.open_button = QPushButton("Ouvrir Fichier CSV")
        self.open_button.setStyleSheet("background-color: #87CEEB;")
        self.open_button.clicked.connect(self.open_file)
        layout.addWidget(self.open_button)

        self.convert_button = QPushButton("Convertir")
        self.convert_button.setStyleSheet("background-color: #87CEEB;")
        self.convert_button.clicked.connect(self.convert_file)
        layout.addWidget(self.convert_button)

        # Section Filtrage
        self.label_filtre = QLabel("Sélectionnez un fichier .xyz à filtrer")
        layout.addWidget(self.label_filtre)

        self.filter_button = QPushButton("Ouvrir Fichier .xyz pour filtrer")
        self.filter_button.setStyleSheet("background-color: #87CEEB;")
        self.filter_button.clicked.connect(self.open_filter_file)
        layout.addWidget(self.filter_button)

        self.slider_label = QLabel("Plage de filtrage : 8000")
        layout.addWidget(self.slider_label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(50)
        self.slider.setMaximum(12000)
        self.slider.setValue(8000)
        self.slider.valueChanged.connect(self.update_slider_value)
        layout.addWidget(self.slider)

        self.filter_and_save_button = QPushButton("Appliquer Filtrage")
        self.filter_and_save_button.setStyleSheet("background-color: #87CEEB;")
        self.filter_and_save_button.clicked.connect(self.apply_filter)
        layout.addWidget(self.filter_and_save_button)

        self.view_button = QPushButton("Visualiser Nuage de Points")
        self.view_button.setStyleSheet("background-color: #87CEEB;")
        self.view_button.clicked.connect(self.visualize_point_cloud)
        layout.addWidget(self.view_button)

        # Section Exemples
        self.example_label = QLabel("Exemples de Nuages de Points:")
        layout.addWidget(self.example_label)

        self.example_list = QListWidget()
        self.example_list.itemClicked.connect(self.view_example_cloud)
        layout.addWidget(self.example_list)

        self.load_example_files()

        # Section Fichiers 3D
        self.stl_label = QLabel("Fichiers 3D:")
        layout.addWidget(self.stl_label)

        self.stl_list = QListWidget()
        self.stl_list.itemClicked.connect(self.view_stl_file)
        layout.addWidget(self.stl_list)

        self.load_stl_files()

        # Configurer le widget central
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Menu Bar
        self.menu_bar = self.menuBar()
        self.credits_action = QAction("Crédits", self)
        self.credits_action.triggered.connect(self.show_credits)
        self.menu_bar.addAction(self.credits_action)

        self.csv_file = ""
        self.xyz_file = ""
        self.converted_file = ""
        self.filtered_file = "" 
        self.range_value = 8000 

    def open_file(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Sélectionnez un fichier CSV", "", "CSV Files (*.csv)", options=options)
        if file_name:
            self.csv_file = file_name
            self.label.setText(f"Fichier sélectionné : {file_name}")
    
    def open_filter_file(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Sélectionnez un fichier .xyz", "", "XYZ Files (*.xyz)", options=options)
        if file_name:
            self.xyz_file = file_name
            self.label_filtre.setText(f"Fichier à filtrer : {file_name}")
    
    def convert_file(self):
        if not self.csv_file:
            QMessageBox.warning(self, "Erreur", "Aucun fichier CSV sélectionné.")
            return
        
        # Créer le nouveau nom de fichier
        base_name = os.path.splitext(self.csv_file)[0]
        self.converted_file = f"{base_name}_conv.xyz"
        
        try:
            with open(self.converted_file, 'w') as f_out:
                with open(self.csv_file, 'r') as file:
                    for line in file:
                        values = list(map(float, line.strip().split(',')))
                        if not conversion(values): 
                            continue
                        x, y, z = conversion(values)
                        f_out.write(f'{x} {y} {z}\n')
            QMessageBox.information(self, "Succès", "Conversion terminée.")
        
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Une erreur s'est produite : {str(e)}")
    
    def apply_filter(self):
        if not self.xyz_file:
            QMessageBox.warning(self, "Erreur", "Aucun fichier .xyz sélectionné pour le filtrage.")
            return
        
        # Créer le nouveau nom de fichier
        base_name = os.path.splitext(self.xyz_file)[0]
        self.filtered_file = f"{base_name}_filtré.xyz"
        
        try:
            with open(self.xyz_file, 'r') as f_in:
                with open(self.filtered_file, 'w') as f_out:
                    for line in f_in:
                        values = line.strip().split()
                        if len(values) != 3:
                            continue
                        x, y, z = map(float, values)
                        if traitement((x, y, z), self.range_value):
                            f_out.write(line)
            QMessageBox.information(self, "Succès", "Filtrage terminé.")
        
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Une erreur s'est produite : {str(e)}")

    def update_slider_value(self, value):
        self.range_value = value
        self.slider_label.setText(f"Plage de filtrage : {value}")
    
    def visualize_point_cloud(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Sélectionnez un fichier .xyz à visualiser", "", "XYZ Files (*.xyz)", options=options)
        if not file_name:
            QMessageBox.warning(self, "Erreur", "Aucun fichier .xyz sélectionné pour la visualisation.")
            return
        
        try:
            point_cloud = load_point_cloud(file_name)
            o3d.visualization.draw_geometries([point_cloud])
        
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Impossible de visualiser le nuage de points : {str(e)}")

    def load_example_files(self):
        """Charge tous les fichiers d'exemple dans le dossier Exemples_nuages."""
        example_dir = "Exemples_nuages"
        if not os.path.exists(example_dir):
            QMessageBox.warning(self, "Erreur", f"Le dossier '{example_dir}' n'existe pas.")
            return

        # Liste tous les fichiers .xyz dans le répertoire
        for file_name in os.listdir(example_dir):
            if file_name.endswith(".xyz"):
                self.example_list.addItem(file_name)

    def view_example_cloud(self, item):
        """Affiche le nuage de points lorsque l'utilisateur clique sur un exemple."""
        example_dir = "Exemples_nuages"
        file_path = os.path.join(example_dir, item.text())
        
        try:
            point_cloud = load_point_cloud(file_path)
            o3d.visualization.draw_geometries([point_cloud])
        
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Impossible de visualiser le nuage de points : {str(e)}")

    def load_stl_files(self):
        """Charge tous les fichiers .stl dans le dossier Fichier_3D."""
        stl_dir = "Fichier_3D"
        if not os.path.exists(stl_dir):
            QMessageBox.warning(self, "Erreur", f"Le dossier '{stl_dir}' n'existe pas.")
            return

        # Liste tous les fichiers .stl dans le répertoire
        for file_name in os.listdir(stl_dir):
            if file_name.endswith(".stl"):
                self.stl_list.addItem(file_name)
                
    def view_stl_file(self, item):
        """Affiche le fichier .stl lorsque l'utilisateur clique sur un exemple."""
        stl_dir = "Fichier_3D"
        file_path = os.path.join(stl_dir, item.text())
        
        try:
            # Charger le fichier .stl
            mesh = o3d.io.read_triangle_mesh(file_path)
            if mesh.is_empty():
                raise ValueError("Le fichier .stl est vide ou invalide.")
            
            # Calculer la couleur uniforme pour le maillage
            base_color = np.array([0.7, 0.7, 0.7])
            mesh.paint_uniform_color(base_color)
    
            # Appliquer des Normales (pour le rendu 3D correct)
            mesh.compute_vertex_normals()
    
            # Visualiser le maillage avec une meilleure lumière
            vis = o3d.visualization.Visualizer()
            vis.create_window()
            vis.add_geometry(mesh)
    
            # Configurer l'éclairage
            opt = vis.get_render_option()
            opt.mesh_show_back_face = True
            opt.light_on = True  
            
            # Lancer la visualisation
            vis.run()
            vis.destroy_window()
        
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Impossible de visualiser le fichier .stl : {str(e)}")

    def show_credits(self):
        """Affiche les crédits dans une boîte de dialogue."""
        credits_dialog = CreditsDialog()
        credits_dialog.exec_()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PointCloudApp()
    window.show()
    sys.exit(app.exec_())
