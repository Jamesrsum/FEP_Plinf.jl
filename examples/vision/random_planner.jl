using Base: @kwdef
using Parameters: @unpack
using SymbolicPlanners, Plinf
using SymbolicPlanners: PathNode, simplify_goal, LinkedNodeRef, @auto_hash, @auto_equals, reconstruct
using PDDL

mutable struct FEPPlanner <: Planner
    max_nodes::Int
    max_time::Float64
    save_search::Bool
    save_search_order::Bool
    verbose::Bool
    callback::Union{Nothing, Function}

    FEPPlanner(;max_nodes::Int=typemax(Int), max_time::Float64=Inf,
                save_search::Bool=false, save_search_order::Bool=save_search,
                verbose::Bool=false, callback::Union{Nothing, Function}=verbose ? LoggerCallback() : nothing) =
        new(max_nodes, max_time, save_search, save_search_order, verbose, callback)
end

@auto_hash FEPPlanner
@auto_equals FEPPlanner

function calculate_free_energy(state::State, domain::Domain, goal::Specification)
    predicted_state = predict_state(state, domain, goal)
    actual_state = get_actual_state(state)
    kl_divergence = calculate_kl_divergence(predicted_state, actual_state)
    return kl_divergence
end

function predict_state(state::State, domain::Domain, goal::Specification)
    # Predict the next state based on the current state, domain, and goal
    return state # Placeholder: return the current state as a dummy prediction
end

function get_actual_state(state::State)
    # Get the actual state from the environment or simulation
    return state # Placeholder: return the current state as the actual state
end

function calculate_kl_divergence(predicted_state::State, actual_state::State)
    # Calculate the KL divergence between the predicted and actual states
    return rand() # Placeholder: return a dummy KL divergence
end

# Define how the FEPPlanner is called with a domain, state, and goal
function (planner::FEPPlanner)(domain::Domain, state::State, goal)
    solve(planner, domain, state, Specification(goal))
end

function (planner::FEPPlanner)(domain::Domain, state::State, goal::MinStepsGoal)
    solve(planner, domain, state, Specification(goal.terms))
end

# Define the solving function to use free energy minimization
function solve(planner::FEPPlanner, domain::Domain, state::State, spec::Specification)
    @unpack save_search = planner

    initial_node = PathNode(hash(state), state, 0.0, calculate_free_energy(state, domain, spec), LinkedNodeRef(nothing))
    search_tree = Dict(initial_node.id => initial_node)
    queue = [initial_node.id]
    search_order = UInt[]
    sol = PathSearchSolution(:in_progress, Term[], Vector{typeof(state)}(), 0, search_tree, queue, search_order)

    sol = search!(sol, planner, domain, spec, state)

    if save_search
        return sol
    elseif sol.status == :failure
        return NullSolution(sol.status)
    else
        return PathSearchSolution(sol.status, sol.plan, sol.trajectory)
    end
end

function search!(sol::PathSearchSolution, planner::FEPPlanner,
                 domain::Domain, spec::Specification, state::State)
    start_time = time()
    sol.expanded = 0
    queue, search_tree = sol.search_frontier, sol.search_tree
    visited = Set{Tuple{Int64,Int64}}()
    initial_agent_pos = get_agent_pos(state)
    push!(visited, initial_agent_pos)

    while length(queue) > 0
        # Prioritize nodes with the lowest free energy
        node_id = find_min_free_energy_node(queue, search_tree)
        node = search_tree[node_id]

        if is_goal(spec, domain, node.state, node.parent.action)
            sol.status = :success # Goal reached
        elseif sol.expanded >= planner.max_nodes
            sol.status = :max_nodes # Node budget reached
        elseif time() - start_time >= planner.max_time
            sol.status = :max_time # Time budget reached
        end

        if sol.status == :in_progress # Expand current node
            deleteat!(queue, findfirst(x -> x == node_id, queue))
            expand!(planner, node, search_tree, queue, domain, spec, visited)
            sol.expanded += 1
            if planner.save_search && planner.save_search_order
                push!(sol.search_order, node_id)
            end
            if !isnothing(planner.callback)
                planner.callback(planner, sol, node_id, sol.expanded)
            end
        else # Reconstruct plan and return solution
            sol.plan, sol.trajectory = reconstruct(node_id, search_tree)
            if !isnothing(planner.callback)
                planner.callback(planner, sol, node_id, sol.expanded)
            end
            return sol
        end
    end
    sol.status = :failure
    return sol
end

function expand!(planner::FEPPlanner, node::PathNode{S},
                 search_tree::Dict{UInt,PathNode{S}}, queue::Vector{UInt},
                 domain::Domain, spec::Specification, visited::Set{Tuple{Int64,Int64}})
                 where {S <: State}
    state = node.state
    available_actions = [act for act in PDDL.available(domain, state)]
    if isempty(available_actions)
        return  # Return if there is no direction to proceed
    end

    for act in available_actions
        next_state = PDDL.transition(domain, state, act, check=false)
        next_id = hash(next_state)
        next_free_energy = calculate_free_energy(next_state, domain, spec)
        
        if !haskey(search_tree, next_id)
            path_cost = node.path_cost + 1
            search_tree[next_id] = PathNode(next_id, next_state, path_cost, next_free_energy, LinkedNodeRef(node.id, act))
            push!(queue, next_id)
        end
    end
end

function find_min_free_energy_node(queue::Vector{UInt}, search_tree::Dict{UInt,PathNode})
    # Find the node with the minimum free energy in the queue
    min_free_energy = Inf
    min_node_id = queue[1]
    for node_id in queue
        node = search_tree[node_id]
        if node.free_energy < min_free_energy
            min_free_energy = node.free_energy
            min_node_id = node_id
        end
    end
    return min_node_id
end