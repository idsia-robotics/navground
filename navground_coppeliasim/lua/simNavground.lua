local simNavground = {}

if loadPlugin then
	simNavground = loadPlugin 'simNavground';
end

function set(property, value)
	if property.type == simNavground.PropertyType.bool then
		property.bool_value = value
	elseif property.type == simNavground.PropertyType.int then
		property.int_value = value
	elseif property.type == simNavground.PropertyType.float then
		property.float_value = value
	elseif property.type == simNavground.PropertyType.string then
		property.string_value = value
	elseif property.type == simNavground.PropertyType.vector then
		property.vector_value = value
	elseif property.type == simNavground.PropertyType.bool_list then
		property.bool_list = value
	elseif property.type == simNavground.PropertyType.int_list then
		property.int_list = value
	elseif property.type == simNavground.PropertyType.float_list then
		property.float_list = value
	elseif property.type == simNavground.PropertyType.string_list then
		property.string_list = value
	elseif property.type == simNavground.PropertyType.vector_list then
		property.vector_list = value
	end
	return property
end
function get(property)
	if property.type == simNavground.PropertyType.bool then
		return property.bool_value
	elseif property.type == simNavground.PropertyType.int then
		return property.int_value
	elseif property.type == simNavground.PropertyType.float then
		return property.float_value
	elseif property.type == simNavground.PropertyType.string then
		return property.string_value
	elseif property.type == simNavground.PropertyType.vector then
		return property.vector_value
	elseif property.type == simNavground.PropertyType.bool_list then
		return property.bool_list
	elseif property.type == simNavground.PropertyType.int_list then
		return property.int_list
	elseif property.type == simNavground.PropertyType.float_list then
		return property.float_list
	elseif property.type == simNavground.PropertyType.string_list then
		return property.string_list
	elseif property.type == simNavground.PropertyType.vector_list then
		return property.vector_list
	end
	return nil
end
function simNavground.set_property(agent, owner, name, value)
	local p = simNavground._get_property(agent, owner, name)
	p = set(p, value)
	simNavground._set_property(agent, owner, name, p)
end

function simNavground.get_property(agent, owner, name)
	local p = simNavground._get_property(agent, owner, name)
	return get(p)
end

return simNavground
