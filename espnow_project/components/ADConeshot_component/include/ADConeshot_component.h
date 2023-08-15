
/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
#define READ_LEN   256
#define ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define FILTER_LEN  15
#define ADC_ATTEN           ADC_ATTEN_DB_6
/*---------------------------------------------------------------
        GPIO General Macros
---------------------------------------------------------------*/
#define BLINK_GPIO 10
#define TRIGGER_ADC_PIN 6


typedef struct{
	int adc_raw; 					/*4 bytes*/
	int voltage;					/*4 bytes*/
	uint32_t adc_buff[FILTER_LEN];	/*4 bytes*/
	int sum;						/*4 bytes*/
	uint32_t adc_filtered;
	int AN_i;						/*4 bytes*/
}struct_adcread;

typedef struct{
	int num;
	struct_adcread *adc_read;
}struct_adclist;
//static_assert(sizeof(struct_adclist) == 8);

uint32_t read_adc_avg(struct_adclist *ADC_Raw, int chn);
uint32_t get_adc_filtered_read(struct_adclist *my_reads, int adc_ch);
uint32_t get_adc_voltage_read(struct_adclist *my_reads, int adc_ch);
esp_err_t adc_init(struct_adclist *my_reads);
esp_err_t set_adc_channel(int* ptr, int size);
