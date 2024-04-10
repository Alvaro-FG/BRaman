from abc import ABC, abstractmethod

class BRamanSpectrometerController(ABC):
    pass

    def get_spectrum_df(self):
        if self.spectro_model == 'EAGLE' and self.exc_wl_nm == 785:
            df = pd.read_csv(os.path.join(os.path.dirname(os.path.abspath(__file__)),'./B_Raman_Calibration/EAGLE_PixeltoLambdatoCM_1.csv'))
            df['Intensity'] = self.get_raw_spectrum()
            df = df.drop('Excitation', axis=1)
        else:
            raise Exception(f'Unrecognized spectrometer model {self.spectro_model}')
        return df