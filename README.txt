# GUID_SEQ
GUID_SEQ code

Le module SEQ réalise le séquencement de la trajectoire et les calculs de XTK, TAE et alongpathdistance pour les envoyer aux autres modules.
Quand le vol est lancé et lorsqu'une position d'avion est reçue (fichier recep_msg.py), les fonctions de séquencement du path (path_sequencing( )) 
et de la trajectoire (sequencing_conditions( )) sont exécutées pour savoir où se situe l'avion par rapport à la trajectoire.
Ensuite les fonctions de calculs (xtk(), tae(), alongpath_distance()) sont exécutées et les résultats sont envoyés sur le bus IVY.

Environnement : 
- télécharger anaconda3
- télécharger python3.8
- télécharger le module ivy.std_api
- télécharger ivyprobe disponible pour Windows sur https://www.eei.cena.fr/products/ivy/download/binaries.html#windows 

Pour lancer l'application :
- Exécuter le main.py

!! Pour relancer le main.py, il faut créer une nouvelle console python pour se déconnecter du bus IVY
