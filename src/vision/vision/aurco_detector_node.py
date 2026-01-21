#!/usr/bin/env python3
"""
Détection de marqueurs ArUco personnalisés avec Raspberry Pi Camera
Utilise des patterns spécifiques pour identifier JAUNE, BLEU et NOIR
Compatible OpenCV 4.6.0
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import time
import sys

# DÉFINITION DES PATTERNS PERSONNALISÉS (grille 6x6)
# 1 = noir, 0 = blanc
# Format: [ligne1, ligne2, ligne3, ligne4, ligne5, ligne6]
# Chaque ligne est une liste de 6 bits

CUSTOM_PATTERNS = {
    "JAUNE": np.array([
        [1, 1, 1, 1, 1, 1],  # Ligne 1: remplie de noir
        [1, 1, 0, 1, 1, 1],  # Ligne 2: noir sauf case 3
        [1, 0, 1, 0, 0, 1],  # Ligne 3: noir sauf cases 2, 4, 5
        [1, 1, 0, 0, 1, 1],  # Ligne 4: noir sauf cases 3, 4
        [1, 1, 0, 1, 1, 1],  # Ligne 5: noir sauf case 3
        [1, 1, 1, 1, 1, 1]   # Ligne 6: remplie de noir
    ], dtype=np.uint8),
    
    "BLEU": np.array([
        [1, 1, 1, 1, 1, 1],  # Ligne 1: remplie de noir
        [1, 1, 1, 1, 0, 1],  # Ligne 2: noir sauf case 5
        [1, 0, 1, 1, 1, 1],  # Ligne 3: noir sauf case 2
        [1, 1, 0, 0, 0, 1],  # Ligne 4: noir sauf cases 3, 4, 5
        [1, 1, 0, 1, 0, 1],  # Ligne 5: noir sauf cases 3, 5
        [1, 1, 1, 1, 1, 1]   # Ligne 6: remplie de noir
    ], dtype=np.uint8),
    
    "NOIR": np.array([
        [1, 1, 1, 1, 1, 1],  # Ligne 1: remplie de noir
        [1, 1, 1, 0, 1, 1],  # Ligne 2: noir sauf case 4
        [1, 0, 1, 0, 1, 1],  # Ligne 3: noir sauf cases 2, 4
        [1, 1, 1, 0, 1, 1],  # Ligne 4: noir sauf case 4
        [1, 0, 1, 1, 1, 1],  # Ligne 5: noir sauf case 2
        [1, 1, 1, 1, 1, 1]   # Ligne 6: remplie de noir
    ], dtype=np.uint8)
}

# Couleurs d'affichage pour chaque pattern
PATTERN_COLORS = {
    "JAUNE": (0, 215, 255),    # BGR
    "BLEU": (255, 100, 0),
    "NOIR": (128, 128, 128)
}

def init_camera():
    """Initialise la caméra Raspberry Pi"""
    print("Initialisation de la caméra...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (1980, 1080), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    print("✓ Caméra prête\n")
    return picam2

def extract_marker_pattern(gray_img, corners):
    """
    Extrait le pattern d'un marqueur ArUco détecté
    
    Args:
        gray_img: Image en niveaux de gris
        corners: Coordonnées des 4 coins du marqueur
        
    Returns:
        pattern: Matrice 6x6 du pattern (0=blanc, 1=noir)
    """
    # Obtenir les 4 coins
    pts = corners.reshape(4, 2)
    
    # Définir les coordonnées de destination pour la transformation (carré 60x60)
    dst_size = 60
    dst_pts = np.array([
        [0, 0],
        [dst_size, 0],
        [dst_size, dst_size],
        [0, dst_size]
    ], dtype=np.float32)
    
    # Calculer la matrice de transformation perspective
    matrix = cv2.getPerspectiveTransform(pts.astype(np.float32), dst_pts)
    
    # Appliquer la transformation
    warped = cv2.warpPerspective(gray_img, matrix, (dst_size, dst_size))
    
    # Binariser l'image (seuil adaptatif)
    _, binary = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Extraire la grille 6x6 (en ignorant la bordure)
    cell_size = dst_size // 6
    pattern = np.zeros((6, 6), dtype=np.uint8)
    
    for i in range(6):
        for j in range(6):
            # Calculer le centre de chaque cellule
            y = i * cell_size + cell_size // 2
            x = j * cell_size + cell_size // 2
            
            # Échantillonner plusieurs pixels autour du centre
            sample_region = binary[
                max(0, y-2):min(dst_size, y+3),
                max(0, x-2):min(dst_size, x+3)
            ]
            
            # Moyenne de la région: si < 128 -> noir (1), sinon blanc (0)
            pattern[i, j] = 1 if np.mean(sample_region) < 128 else 0
    
    return pattern

def match_pattern(pattern):
    """
    Compare le pattern extrait avec les patterns personnalisés
    
    Args:
        pattern: Matrice 6x6 extraite
        
    Returns:
        (nom_couleur, couleur_BGR, score_confiance) ou (None, None, 0)
    """
    best_match = None
    best_score = 0
    best_color = None
    
    for color_name, ref_pattern in CUSTOM_PATTERNS.items():
        # Tester les 4 rotations possibles
        for rotation in range(4):
            rotated = np.rot90(pattern, rotation)
            
            # Calculer le score de correspondance (pourcentage de pixels identiques)
            matches = np.sum(rotated == ref_pattern)
            score = matches / 36.0  # 36 cellules au total
            
            if score > best_score:
                best_score = score
                best_match = color_name
                best_color = PATTERN_COLORS[color_name]
    
    # Seuil de confiance minimum (85%)
    if best_score >= 0.85:
        return best_match, best_color, best_score
    
    return None, None, best_score

def detect_custom_aruco(frame, aruco_dict, aruco_params):
    """
    Détecte les marqueurs ArUco et identifie leur couleur par pattern
    
    Returns:
        detected_markers: Liste de (corners, color_name, color_bgr, confidence)
    """
    # Convertir en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # Détecter tous les marqueurs ArUco avec l'ancienne API
    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=aruco_params
    )
    
    detected_markers = []
    
    if corners is not None and len(corners) > 0:
        for i, corner in enumerate(corners):
            # Extraire le pattern du marqueur
            pattern = extract_marker_pattern(gray, corner[0])
            
            # Identifier le pattern
            color_name, color_bgr, confidence = match_pattern(pattern)
            
            if color_name:
                detected_markers.append({
                    'corners': corner[0],
                    'color_name': color_name,
                    'color_bgr': color_bgr,
                    'confidence': confidence,
                    'pattern': pattern
                })
    
    return detected_markers

def draw_markers_info(frame, detected_markers):
    """Dessine les marqueurs détectés avec leurs informations"""
    frame_annotated = frame.copy()
    
    for marker in detected_markers:
        corner = marker['corners']
        color_name = marker['color_name']
        color_bgr = marker['color_bgr']
        confidence = marker['confidence']
        
        # Calculer le centre
        center_x = int(np.mean(corner[:, 0]))
        center_y = int(np.mean(corner[:, 1]))
        
        # Dessiner le contour
        pts = corner.astype(np.int32)
        cv2.polylines(frame_annotated, [pts], True, color_bgr, 3)
        
        # Dessiner les coins
        for point in pts:
            cv2.circle(frame_annotated, tuple(point), 8, color_bgr, -1)
        
        # Dessiner le centre
        cv2.circle(frame_annotated, (center_x, center_y), 10, color_bgr, -1)
        cv2.circle(frame_annotated, (center_x, center_y), 12, (255, 255, 255), 2)
        
        # Calculer la taille
        width = np.linalg.norm(corner[0] - corner[1])
        height = np.linalg.norm(corner[1] - corner[2])
        size = int((width + height) / 2)
        
        # Texte avec informations
        text_color = f"{color_name}"
        text_conf = f"{confidence*100:.0f}%"
        text_size = f"{size}px"
        
        # Fond semi-transparent
        text_bg_height = 85
        text_bg_width = 150
        overlay = frame_annotated.copy()
        cv2.rectangle(overlay, 
                     (center_x - text_bg_width//2, center_y - 50),
                     (center_x + text_bg_width//2, center_y - 50 + text_bg_height),
                     (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame_annotated, 0.4, 0, frame_annotated)
        
        # Dessiner le texte
        cv2.putText(frame_annotated, text_color,
                   (center_x - 60, center_y - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_bgr, 2)
        
        cv2.putText(frame_annotated, text_conf,
                   (center_x - 35, center_y + 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(frame_annotated, text_size,
                   (center_x - 30, center_y + 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    return frame_annotated

def print_detection_info(detected_markers, detection_count, frame_count):
    """Affiche les informations de détection"""
    print(f"\n{'='*60}")
    print(f"DÉTECTION #{detection_count} - Frame {frame_count}")
    print(f"{'='*60}")
    print(f"Nombre de marqueurs détectés: {len(detected_markers)}\n")
    
    for i, marker in enumerate(detected_markers):
        corner = marker['corners']
        color_name = marker['color_name']
        confidence = marker['confidence']
        pattern = marker['pattern']
        
        center_x = int(np.mean(corner[:, 0]))
        center_y = int(np.mean(corner[:, 1]))
        
        width = np.linalg.norm(corner[0] - corner[1])
        height = np.linalg.norm(corner[1] - corner[2])
        size = int((width + height) / 2)
        
        print(f"  Marqueur {i+1} - {color_name} (confiance: {confidence*100:.1f}%):")
        print(f"    • Position: ({center_x:4d}, {center_y:4d})")
        print(f"    • Taille:   ~{size}px")
        print(f"    • Pattern détecté:")
        for row in pattern:
            print(f"      {' '.join(['█' if x == 1 else '░' for x in row])}")
        print()

def main():
    """Fonction principale"""
    print("="*60)
    print("DÉTECTION DE MARQUEURS ARUCO PERSONNALISÉS")
    print("="*60)
    print(f"Version OpenCV: {cv2.__version__}")
    print("\nPatterns recherchés:")
    for color_name in CUSTOM_PATTERNS.keys():
        print(f"  • {color_name}")
    print("\nAppuyez sur Ctrl+C pour arrêter\n")
    
    # Initialiser le détecteur ArUco
    print("Initialisation du détecteur ArUco...")
    try:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        print("✓ Détecteur prêt\n")
    except Exception as e:
        print(f"❌ Erreur création détecteur: {e}")
        sys.exit(1)
    
    # Initialiser la caméra
    try:
        picam2 = init_camera()
    except Exception as e:
        print(f"❌ Erreur d'initialisation de la caméra: {e}")
        sys.exit(1)
    
    frame_count = 0
    detection_count = 0
    last_markers = []
    
    try:
        print("🔍 Scan en cours...\n")
        
        while True:
            frame = picam2.capture_array()
            frame_count += 1
            
            # Détecter les marqueurs personnalisés
            detected_markers = detect_custom_aruco(frame, aruco_dict, aruco_params)
            
            if detected_markers:
                # Créer une signature de la détection actuelle
                current_sig = tuple(sorted([m['color_name'] for m in detected_markers]))
                last_sig = tuple(sorted([m['color_name'] for m in last_markers]))
                
                # Nouvelle détection ou changement
                if current_sig != last_sig:
                    detection_count += 1
                    
                    # Afficher les informations
                    print_detection_info(detected_markers, detection_count, frame_count)
                    
                    # Dessiner sur l'image
                    frame_annotated = draw_markers_info(frame, detected_markers)
                    
                    # Sauvegarder
                    filename = f"aruco_custom_{detection_count:03d}.jpg"
                    cv2.imwrite(filename, cv2.cvtColor(frame_annotated, cv2.COLOR_RGB2BGR))
                    print(f"  💾 Image sauvegardée: {filename}")
                    
                    last_markers = detected_markers
            else:
                if last_markers:
                    print(f"[Frame {frame_count}] Aucun marqueur visible")
                last_markers = []
            
            if frame_count % 100 == 0:
                print(f"[Frame {frame_count}] En cours... (Détections: {detection_count})")
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("ARRÊT DU PROGRAMME")
        print("="*60)
        print(f"Frames analysées:     {frame_count}")
        print(f"Détections effectuées: {detection_count}")
        print(f"\n✓ Images sauvegardées: aruco_custom_*.jpg")
    
    except Exception as e:
        print(f"\n❌ Erreur: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        picam2.stop()
        print("\n✓ Caméra arrêtée proprement")

if __name__ == "__main__":
    main()