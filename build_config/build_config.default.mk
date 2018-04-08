#####       Configuration du build pour l'asservissement       #####

##########################################################
# Attention: Fichier de config par défaut, à copier vers #
#           build_config.mk et à modifier                #
##########################################################

# Syntaxe : Pour désactiver une option, commenter la ligne. Exemple:
# FOO_BAR is not set
# Pour activer une option, décommenter, et assigner une valeur. Exemple:
FOO_BAR=1
# *NE PAS* mettre d'espaces autour du signe '=' !
# Dans le code C, le paramètre sera défini comme une macro préprocesseur
# dont le nom est préfixé par "CONFIG_". Par exemple, ici, la macro
# CONFIG_FOO_BAR sera définie à 1.

# Ci-dessous les options modifiables:
# Si des options sont rajoutés, penser à les ajouter aussi au fichier
# de config par défaut.

# COM_I2C_ACTIVATE is not set
# DEBUG_COM_I2C is not set
# LCD_ACTIVATE is not set
COM_SERIE_ACTIVATE=1
# COM_SERIEPC_ACTIVATE is not set

# Type des codeurs. UN SEUL TYPE doit être activé, sinon des choses
# étranges vont se produire
CODEUR_DIRECTS=1
# CODEUR_AVR is not set

# Type des codeurs. Idem, UN SEUL TYPE doit être activé.
MOTORCTRL_MD22=1
# MOTORCTRL_MD25 is not set
# MOTORCTRL_QIK is not set
# MOTORCTRL_POLOLU_SMCS is not set
