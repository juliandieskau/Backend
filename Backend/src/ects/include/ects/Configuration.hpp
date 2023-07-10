namespace ects {
class Configuration {
public:
  Configuration();
  ~Configuration();
};
auto load_configuration() -> Configuration *;
} // namespace ects