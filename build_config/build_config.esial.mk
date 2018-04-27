#####       Configuration du build pour l'asservissement       #####

##########################################################
#                 Config pour l'ESIAL                    #
##########################################################

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
