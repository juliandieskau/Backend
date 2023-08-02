#include "UsageMonitor.hpp"

namespace ects::plugins::systemmonitor {

template <typename T> auto UsageDataCollection<T>::add(T data) -> void {
    current_aggregation.push_back(data);
    // NOTE: this implementation saves all data till it aggregates,
    //       requires a default constructor uses operators += and /=
    auto average = [this] {
        T acc;
        for (auto d : current_aggregation)
            acc += d;
        acc /= current_aggregation.size();
        current_aggregation.clear();
        return acc;
    };
    if (strategy->new_data(&data))
        history.add(average());
}

template <typename T>
auto UsageDataCollection<T>::to_ros(UsageDataCollection<T> collection)
    -> ros_t {
    ros_t r;
    r.aggregation = collection.strategy.to_ros();
    r.measurements = collection.history;
    return r;
}

template <typename T>
UsageDataMonitor<T>::UsageDataMonitor(
    std::string topic_name,
    const std::vector<AggregationStrategy *> &aggregation_strategies,
    std::unique_ptr<UsageProvider<T>> data_provider)
    : topic_name(std::move(topic_name)), data_provider(data_provider),
      collected_usage_data(), collection_publishers(),
      last_usage_publisher(last_usage_topic_name + topic_name),
      size(aggregation_strategies.size()) {
    for (auto aggregation : aggregation_strategies) {
        collected_usage_data.emplace_back(aggregation);
        collection_publishers.emplace_back(aggregation_topic_name +
                                           aggregation->get_name() +
                                           this->topic_name);
    }
}

template <typename T> auto UsageDataMonitor<T>::step() -> void {
    T data = data_provider->get_usage();
    last_usage_publisher.publish(data);
    for (size_t i = 0; i < size; ++i) {
        collected_usage_data[i].add(data);
        collection_publishers[i].publish(collected_usage_data[i]);
    }
}

} // namespace ects::plugins::systemmonitor