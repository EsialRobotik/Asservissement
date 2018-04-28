#####       Configuration du build pour l'asservissement       #####

##########################################################
#                    Config pour PMX                     #
##########################################################

# Si des options sont rajoutés, penser à les ajouter aussi au fichier
# de config par défaut.

COM_I2C_ACTIVATE=1
DEBUG_COM_I2C=1
# LCD_ACTIVATE is not set
# COM_SERIE_ACTIVATE is not set
COM_SERIEPC_ACTIVATE=1

# Type des codeurs. UN SEUL TYPE doit être activé, sinon des choses
# étranges vont se produire
CODEUR_DIRECTS=1
# CODEUR_AVR is not set

# Type des codeurs. Idem, UN SEUL TYPE doit être activé.
# MOTORCTRL_MD22 is not set
MOTORCTRL_MD25=1
# MOTORCTRL_QIK is not set
# MOTORCTRL_POLOLU_SMCS is not set
