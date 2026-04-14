"""
tools/calibrate.py

Génère le fichier config/calibration.npz à partir de photos d'échiquier.

Usage :
    python3 tools/calibrate.py

Prérequis :
    - Mettre 20-25 photos JPG/PNG dans tools/calib_images/
    - L'échiquier imprimé doit avoir 9x6 coins intérieurs (grille 10x7)
    - Mesurer la taille réelle d'un carré et la passer avec --size

Options :
    --images  : dossier des photos         (défaut: tools/calib_images)
    --size    : taille d'un carré en m     (défaut: 0.025 = 2.5cm)
    --grid    : coins intérieurs colsxrows (défaut: 9x6)
    --out     : fichier de sortie          (défaut: src/vision/config/calibration.npz)
    --preview : affiche chaque image avec les coins détectés
"""

import cv2
import numpy as np
import glob
import argparse
import os


def main():
    parser = argparse.ArgumentParser(description='Calibration caméra par échiquier')
    parser.add_argument('--images',  default='tools/calib_images',
                        help='Dossier contenant les photos')
    parser.add_argument('--size',    type=float, default=0.025,
                        help='Taille d\'un carré en mètres (ex: 0.025 = 2.5cm)')
    parser.add_argument('--grid',    default='9x6',
                        help='Coins intérieurs : colonnesxlignes (ex: 9x6)')
    parser.add_argument('--out',     default='src/vision/config/calibration.npz',
                        help='Chemin du fichier de sortie')
    parser.add_argument('--preview', action='store_true',
                        help='Affiche les coins détectés sur chaque image')
    args = parser.parse_args()

    # ----------------------------------------------------------------
    # Préparation
    # ----------------------------------------------------------------
    cols, rows = map(int, args.grid.split('x'))

    # Points 3D de l'échiquier dans son repère propre (z=0)
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.size

    objpoints = []   # points 3D réels
    imgpoints = []   # points 2D dans l'image
    img_size  = None

    # ----------------------------------------------------------------
    # Chargement des images
    # ----------------------------------------------------------------
    images = sorted(
        glob.glob(os.path.join(args.images, '*.jpg')) +
        glob.glob(os.path.join(args.images, '*.jpeg')) +
        glob.glob(os.path.join(args.images, '*.png'))
    )

    if not images:
        print(f'[ERREUR] Aucune image trouvée dans {args.images}/')
        print('  → Mets des photos JPG/PNG dans ce dossier et relance.')
        return

    print(f'Images trouvées : {len(images)}')
    print(f'Grille : {cols}x{rows} coins intérieurs')
    print(f'Taille carré : {args.size*100:.1f} cm')
    print()

    valid = 0
    for fname in images:
        img  = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

        if ret:
            # Affinage subpixel
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30, 0.001
                )
            )
            objpoints.append(objp)
            imgpoints.append(corners2)
            img_size = gray.shape[::-1]
            valid += 1
            print(f'  ✓  {os.path.basename(fname)}')

            if args.preview:
                cv2.drawChessboardCorners(img, (cols, rows), corners2, ret)
                cv2.imshow('Calibration preview — appuie sur une touche', img)
                cv2.waitKey(300)
        else:
            print(f'  ✗  {os.path.basename(fname)}  (échiquier non détecté)')

    if args.preview:
        cv2.destroyAllWindows()

    print(f'\nImages valides : {valid}/{len(images)}')

    # ----------------------------------------------------------------
    # Vérification minimum
    # ----------------------------------------------------------------
    if valid < 10:
        print()
        print('[ERREUR] Pas assez d\'images valides (minimum 10).')
        print('Conseils :')
        print('  - Vérifie que l\'échiquier est bien imprimé et plat')
        print('  - Utilise --grid si ta grille n\'est pas 9x6')
        print('  - Varie les angles et distances')
        return

    # ----------------------------------------------------------------
    # Calibration
    # ----------------------------------------------------------------
    print('\nCalibration en cours...')
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None
    )

    print(f'Erreur de reprojection : {ret:.4f} px')
    if ret < 0.5:
        print('  → Excellente calibration ✓')
    elif ret < 1.0:
        print('  → Calibration correcte ✓')
    else:
        print('  → Erreur élevée — essaie avec de meilleures photos')

    print()
    print('Matrice caméra :')
    print(camera_matrix)
    print()
    print('Coefficients de distorsion :')
    print(dist_coeffs)

    # ----------------------------------------------------------------
    # Sauvegarde
    # ----------------------------------------------------------------
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    np.savez(args.out, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    print(f'\nCalibration sauvegardée → {args.out}')
    print('Tu peux maintenant lancer box_detector_node.')


if __name__ == '__main__':
    main()