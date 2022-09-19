#ifndef KMEANS_H_
#define KMEANS_H_

#include <stdio.h>
#include <stdlib.h>

struct record {
  int     rssi_rle[RUN_LENGTH][2];
  uint8_t sequence_num;
};

/**
 * @brief Function for k-means clustering algorithm, used for jamming attacks.
 *
 * @param[in] record Pointer to the struct containing rssi_rle and sequence_numbers.
 */
int kmeans(struct record *, int rle_ptr);

/**
 * @brief Function for k-means clustering algorithm, used for bluetooth and WiFi. (Follows SpeckSense closely).
 *
 * @param[in] record Pointer to the struct containing rssi_rle and sequence_numbers.
 */
int kmeans_old(struct record *, int rle_ptr);


float channel_metric_rssi_threshold(struct record *record, int rle_ptr);
void channel_rate(struct record *record, int n_clusters);
void add_to_tlist(uint16_t ts_diff);
void update_tlist(uint16_t ts_diff);
uint16_t abs_diff(uint16_t a, uint16_t b);

/**
 * @brief Function for comparing the power level and duration with different jamming attacks .
 */
void check_similarity(int profiling);

/**
 * @brief Function for calculating the consistency. (Used for finding random jamming attacks).
 */
void calc_consistency(void);

/*Statistics func*/
// void calc_profiling(void);
// void calc_accuracy(void);

/**
 * @brief Function for comparing the power level and duration with BT and WiFi thresholds.
 *
 * @param[in] record num_clusters created from the rssi-sampler.
 */
void check_unintentional_interference(int num_clusters);

/**
 * @brief Function for calculating and printing the interarrival.
 *
 * @param[in] itr_cnt RADIO_CHANNEL,.
 *
 * @param[in] num_clusters num_clusters created from the rssi-sampler.
 */
void print_interarrival(uint16_t itr_cnt, int num_clusters);
// void print_cluster_centroids(void);

// enum intf_types {WIFI_BEACON = 0, DATA, BLUETOOTH_SINGLE_SLOT, BLUETOOTH_MULTI_SLOT, UNKNOWN};

#endif /* KMEANS_H_ */
